#include <linux/module.h>
#include <linux/printk.h>
#include <linux/usb.h>

#include "./ft9201.h"

MODULE_AUTHOR("Mak Krnic <mak@banianitc.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FT9201 Fingeprint reader driver");

#define VENDOR_ID 0x2808
#define PRODUCT_ID 0x9338

static struct usb_device_id ft9201_table[] = {
		{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
		{}
};
MODULE_DEVICE_TABLE(usb, ft9201_table);

#define FT9201_MINOR_BASE	192

#define USB_CONTROL_OP_TIMEOUT 1000
#define USB_READ_OP_TIMEOUT 1000

#define FT9201_IMG_WIDTH  64
#define FT9201_IMG_HEIGHT 80
#define FT9201_IMG_SIZE   (FT9201_IMG_WIDTH * FT9201_IMG_HEIGHT)

/*
 * USB protocol (observed on Windows driver 2.0.3.83 via USBPcap):
 *
 * - Status poll:  control IN  bmRequestType=0xc0 bRequest=0x43 wValue=0 wIndex=0
 *                 wLength=4; response is 4 bytes.  byte0 == 0x01 means "data
 *                 ready" (finger present / capture available); 0x00 means busy.
 *
 * - Register/block read: every read is preceded by the same two-control setup:
 *                 0x40 0x34 wValue=0x00ff wIndex=0
 *                 0x40 0x34 wValue=0x0003 wIndex=0
 *                 then 0x40 0x6f wValue=<length> wIndex=<address>,
 *                 followed by a bulk IN transfer of <length> bytes.
 *
 * - Command (no data): 0x40 0x6f wValue=0x0000 wIndex=0xff00 - used to arm
 *                 another finger capture after readiness.
 *
 * - Registers:
 *                 0x9180 info/status block (32 bytes, incl. "20220729" and
 *                        fingerprint flag at offset 0)
 *                 0x9180 finger status      (4 bytes; 0x00111100 = present,
 *                        0x00001100 = absent)
 *                 0x9080 quality data       (6 bytes)
 *                 0x9080 raw image          (5120 bytes = 64x80)
 */

// IN Requests
#define FT9201_REQ_GET_SENSOR_INT_PORT_STATES 0x43
#define FT9201_REQ_INIT_READ_ID 0x1a
#define FT9201_REQ_INIT_BOOT_STATUS 0x3a

// OUT Requests
#define FT9201_REQ_START_CAPTURE_PROBABLY 0x34
#define FT9201_REQ_READ_BLOCK_PROBABLY 0x6f
#define FT9201_REQ_INIT_BOOT 0x22

#define FT9201_INIT_MAX_POLLS 6

#define FT9201_INIT_READY_BYTE0 0x0a
#define FT9201_INIT_READY_BYTE1 0x0a

#define FT9201_REG_FINGER_STATUS 0x9180
#define FT9201_REG_IMAGE_BLOCK 0x9080
#define FT9201_REG_TRIGGER_CAPTURE 0xff00

#define FT9201_FINGER_PRESENT 0x00111100
#define FT9201_FINGER_ABSENT 0x00001100

struct ft9201_device {
	struct usb_device *udev;
	struct usb_interface *interface;
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	unsigned long		disconnected:1;

	struct ft9201_status device_status;

	bool            ongoing_read;           /* a read is going on */
	unsigned char   *read_img_data;
	size_t			img_in_size;		/* the size of the receive buffer */
	size_t			img_in_filled;		/* number of bytes in the buffer */
	size_t			img_in_copied;		/* already copied to user space */

};
#define to_ft9201_dev(d) container_of(d, struct ft9201_device, kref)

static int ft9201_open(struct inode *inode, struct file *file);
static int ft9201_release(struct inode *inode, struct file *file);
static long ft9201_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t ft9201_read(struct file *fp, char __user *buf, size_t count, loff_t *f_pos);

static int ft9201_initialize(struct ft9201_device *dev);
static long ft9201_ioctl_get_status(struct ft9201_device *dev, struct ft9201_status *device_status);
static int ft9201_get_sensor_int_port_states(struct ft9201_device *dev, unsigned char *states);
static int ft9201_finger_present(struct ft9201_device *dev, bool *present);
static int ft9201_read_image(struct ft9201_device *dev);
static int ft9201_arm_capture(struct ft9201_device *dev);
static int ft9201_boot_handshake(struct ft9201_device *dev);

static void ft9201_delete(struct kref *kref);

static struct usb_driver ft9201_driver;

static const struct file_operations ft9201_fops = {
		.owner =   THIS_MODULE,
		.llseek =  noop_llseek,
		.open =    ft9201_open,
		.release = ft9201_release,
		.unlocked_ioctl = ft9201_ioctl,
		.read =    ft9201_read,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver ft9201_class = {
		.name =	        "fpreader%d",
		.fops =	        &ft9201_fops,
		.minor_base =   FT9201_MINOR_BASE,
};

/*
 * Every block read on the FT9201 must be preceded by two vendor control
 * writes (0x34/0x00ff then 0x34/0x0003).  This mirrors the sequence the
 * Windows driver issues before each 0x6f access.
 */
static int ft9201_block_read_setup(struct ft9201_device *dev)
{
	int retval;

	retval = usb_control_msg_send(
			dev->udev,
			0,
			FT9201_REQ_START_CAPTURE_PROBABLY,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x00ff,
			0,
			NULL,
			0,
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);
	if (retval) {
		dev_err(&dev->interface->dev, "Block read setup 1 failed: %d", retval);
		return retval;
	}

	retval = usb_control_msg_send(
			dev->udev,
			0,
			FT9201_REQ_START_CAPTURE_PROBABLY,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0003,
			0,
			NULL,
			0,
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);
	if (retval) {
		dev_err(&dev->interface->dev, "Block read setup 2 failed: %d", retval);
		return retval;
	}

	return 0;
}

/*
 * Read `len` bytes from register block `addr`.  The Windows driver transfers
 * exactly `len` bytes on the bulk IN endpoint 0x83 after the 0x6f command
 * whose wValue encodes the byte count and wIndex encodes the address.
 */
static int ft9201_read_block(struct ft9201_device *dev, unsigned short addr,
		unsigned short len, void *buf)
{
	unsigned char *tmp;
	int retval;
	int read_length;

	tmp = kzalloc(len, GFP_KERNEL);
	if (tmp == NULL) {
		return -ENOMEM;
	}

	retval = ft9201_block_read_setup(dev);
	if (retval) {
		goto out;
	}

	retval = usb_control_msg_send(
			dev->udev,
			0,
			FT9201_REQ_READ_BLOCK_PROBABLY,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			len,
			addr,
			NULL,
			0,
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);
	if (retval) {
		dev_err(&dev->interface->dev, "Block read command failed: %d", retval);
		goto out;
	}

	retval = usb_bulk_msg(
			dev->udev,
			usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
			tmp,
			len,
			&read_length,
			USB_READ_OP_TIMEOUT);
	if (retval < 0) {
		dev_err(&dev->interface->dev, "Block read bulk transfer failed: %d", retval);
		goto out;
	}
	if (read_length != len) {
		dev_err(&dev->interface->dev, "Block read short transfer: got %d, want %d",
				read_length, len);
		retval = -EIO;
		goto out;
	}

	memcpy(buf, tmp, len);

out:
	kfree(tmp);
	return retval;
}

/*
 * Read the 32-byte info block at 0x9180.  Its first four bytes carry the same
 * fingerprint flag used by the 4-byte finger-status read.  Windows reads this
 * block whenever the sensor reports ready, before arming a capture.
 */
static int ft9201_read_info_block(struct ft9201_device *dev, void *buf)
{
	return ft9201_read_block(dev, FT9201_REG_FINGER_STATUS, 32, buf);
}

static int ft9201_get_sensor_int_port_states(struct ft9201_device *dev, unsigned char *states)
{
	int retval;
	unsigned char status[4];

	retval = usb_control_msg_recv(
			dev->udev,
			0,
			FT9201_REQ_GET_SENSOR_INT_PORT_STATES,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			0,
			status,
			sizeof(status),
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);

	if (retval) {
		dev_err(&dev->interface->dev, "Error reading sensor states: %d", retval);
		return retval;
	}

	*states = status[0];

	return retval;
}

/*
 * Check whether a finger is present on the sensor.  This reads the 4-byte
 * finger status block at 0x9180: 0x00111100 means present, 0x00001100 absent.
 */
static int ft9201_finger_present(struct ft9201_device *dev, bool *present)
{
	int retval;
	unsigned int finger_status;

	retval = ft9201_read_block(dev, FT9201_REG_FINGER_STATUS, 4, &finger_status);
	if (retval) {
		return retval;
	}

	finger_status = le32_to_cpu(finger_status);

	switch (finger_status) {
	case FT9201_FINGER_PRESENT:
		*present = true;
		break;
	case FT9201_FINGER_ABSENT:
		*present = false;
		break;
	default:
		dev_dbg(&dev->interface->dev, "Unexpected finger status: 0x%08x", finger_status);
		*present = false;
		break;
	}

	return 0;
}

/*
 * Send the "arm another capture" command (0x6f wValue=0 wIndex=0xff00).
 * Windows issues this after the sensor became ready.  No data follows.
 */
static int ft9201_arm_capture(struct ft9201_device *dev)
{
	int retval;

	retval = ft9201_block_read_setup(dev);
	if (retval) {
		return retval;
	}

	retval = usb_control_msg_send(
			dev->udev,
			0,
			FT9201_REQ_READ_BLOCK_PROBABLY,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0000,
			FT9201_REG_TRIGGER_CAPTURE,
			NULL,
			0,
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);
	if (retval) {
		dev_err(&dev->interface->dev, "Capture arm command failed: %d", retval);
		return retval;
	}

	return 0;
}

/*
 * Boot the sensor firmware, mirroring the Windows driver's init handshake
 * observed on a fresh boot (USBPcap success capture, frames 157-176):
 *
 *     IN  0xc0 0x1a wValue=0 wIndex=0 wLength=4   -> 15 04 00 00   (x2)
 *     OUT 0x40 0x22 wValue=0x0070 wIndex=0x0070                   (x2)
 *     IN  0xc0 0x3a wValue=0 wIndex=0x0020 wLength=4 -> 02 02 00 00 (first)
 *     loop x5: { OUT 0x22 0x0070/0x0070 (x2), IN 0x3a 0x0020
 *                -> 0a 0a 00 00 }  (ready)
 *
 * Required when the sensor has not been initialized by a previous Windows
 * session (i.e. after a real power cycle).  Polls until the 0x3a status
 * reads 0a 0a 00 00, then returns.
 */
static int ft9201_boot_handshake(struct ft9201_device *dev)
{
	int retval;
	int poll;
	unsigned char id[4];
	unsigned char status[4];

	/* 1. Two 0x1a ID reads. */
	retval = usb_control_msg_recv(
			dev->udev,
			0,
			FT9201_REQ_INIT_READ_ID,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			0,
			id,
			sizeof(id),
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);
	if (retval) {
		dev_err(&dev->interface->dev, "Init ID read 1 failed: %d", retval);
		return retval;
	}

	retval = usb_control_msg_recv(
			dev->udev,
			0,
			FT9201_REQ_INIT_READ_ID,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			0,
			id,
			sizeof(id),
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);
	if (retval) {
		dev_err(&dev->interface->dev, "Init ID read 2 failed: %d", retval);
		return retval;
	}

	dev_info(&dev->interface->dev, "Init ID: %02x %02x %02x %02x",
			id[0], id[1], id[2], id[3]);

	/* 2. Initial boot write pair, then poll. */
	for (poll = 0; poll <= FT9201_INIT_MAX_POLLS; poll++) {
		retval = usb_control_msg_send(
				dev->udev,
				0,
				FT9201_REQ_INIT_BOOT,
				USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
				0x0070,
				0x0070,
				NULL,
				0,
				USB_CONTROL_OP_TIMEOUT,
				GFP_KERNEL);
		if (retval) {
			dev_err(&dev->interface->dev, "Boot write failed: %d", retval);
			return retval;
		}

		retval = usb_control_msg_send(
				dev->udev,
				0,
				FT9201_REQ_INIT_BOOT,
				USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
				0x0070,
				0x0070,
				NULL,
				0,
				USB_CONTROL_OP_TIMEOUT,
				GFP_KERNEL);
		if (retval) {
			dev_err(&dev->interface->dev, "Boot write failed: %d", retval);
			return retval;
		}

		retval = usb_control_msg_recv(
				dev->udev,
				0,
				FT9201_REQ_INIT_BOOT_STATUS,
				USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
				0,
				0x0020,
				status,
				sizeof(status),
				USB_CONTROL_OP_TIMEOUT,
				GFP_KERNEL);
		if (retval) {
			dev_err(&dev->interface->dev, "Boot status read failed: %d", retval);
			return retval;
		}

		dev_info(&dev->interface->dev, "Boot status poll %d: %02x %02x %02x %02x",
				poll, status[0], status[1], status[2], status[3]);

		if (status[0] == FT9201_INIT_READY_BYTE0 &&
				status[1] == FT9201_INIT_READY_BYTE1) {
			dev_info(&dev->interface->dev, "FT9201 sensor booted");
			return 0;
		}
	}

	dev_err(&dev->interface->dev,
			"FT9201 sensor did not reach ready state after %d polls",
			FT9201_INIT_MAX_POLLS + 1);
	return -ETIMEDOUT;
}

static int ft9201_read_image(struct ft9201_device *dev)
{
	int retval;
	unsigned char *img;

	dev_info(&dev->interface->dev, "Reading image from scanner; dimensions: %dx%d",
			FT9201_IMG_WIDTH, FT9201_IMG_HEIGHT);

	if (dev->read_img_data != NULL) {
		kfree(dev->read_img_data);
		dev->read_img_data = NULL;
		dev->img_in_size = 0;
	}

	img = kzalloc(FT9201_IMG_SIZE, GFP_KERNEL);
	if (img == NULL) {
		return -ENOMEM;
	}

	retval = ft9201_read_block(dev, FT9201_REG_IMAGE_BLOCK, FT9201_IMG_SIZE, img);
	if (retval) {
		kfree(img);
		return retval;
	}

	dev->read_img_data = img;
	dev->img_in_size = FT9201_IMG_SIZE;
	dev->img_in_filled = FT9201_IMG_SIZE;
	dev->img_in_copied = 0;

	return 0;
}

static int ft9201_open(struct inode *inode, struct file *file)
{
	struct usb_interface *intf;
	struct ft9201_device *dev;

	pr_info("ft9201 open\n");

	intf = usb_find_interface(&ft9201_driver, iminor(inode));
	if (!intf) {
		pr_err("Can't find device for minor %d\n", iminor(inode));
		return -ENODEV;
	}

	dev = usb_get_intfdata(intf);
	if (!dev) {
		return -ENODEV;
	}

	kref_get(&dev->kref);

	file->private_data = dev;

	return 0;
}

static int ft9201_release(struct inode *inode, struct file *file)
{
	struct ft9201_device *dev = file->private_data;
	pr_info("ft9201 release\n");

	if (dev == NULL) {
		return -ENODEV;
	}

	kref_put(&dev->kref, ft9201_delete);

	return 0;
}

static int ft9201_initialize(struct ft9201_device *dev)
{
	unsigned char sensor_status;
	int retval;

	retval = ft9201_boot_handshake(dev);
	if (retval < 0) {
		/* Non-fatal: if the sensor is already booted (e.g. a previous
		 * session left it powered), the handshake may time out but the
		 * sensor still works.  Surface the failure and continue. */
		dev_warn(&dev->interface->dev, "Sensor boot handshake failed: %d", retval);
	}

	retval = ft9201_get_sensor_int_port_states(dev, &sensor_status);
	if (retval < 0) {
		dev_err(&dev->interface->dev, "Error getting sensor status");
		return retval;
	}

	dev->device_status.initialized = 1;

	dev_info(&dev->interface->dev, "Device initialization successful (status 0x%02x)",
			sensor_status);
	return 0;
}

static long ft9201_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long errCode;
	struct ft9201_device *dev = file->private_data;

	pr_info("ft9201 ioctl, cmd: %u\n", cmd);

	errCode = 0;

	switch (cmd) {
		case FT9201_IOCTL_REQ_INITIALIZE:
			errCode = ft9201_initialize(dev);
			if (errCode < 0) {
				dev_err(&dev->interface->dev, "Error initializing device: %ld", errCode);
				return errCode;
			}
			break;
		case FT9201_IOCTL_REQ_GET_STATUS:
			if ((void*)arg == NULL) {
				return -EFAULT;
			}

			errCode = ft9201_ioctl_get_status(dev, (struct ft9201_status*) arg);
			if (errCode < 0) {
				dev_err(&dev->interface->dev, "Error getting device status: %ld", errCode);
				return errCode;
			}
			break;

		case FT9201_IOCTL_REQ_SET_AUTO_POWER:
			/* The Windows driver does not run any auto-power register
			 * sequence for this device, so this is a no-op. */
			break;

		case FT9201_IOCTL_REQ_SENSOR_STATUS:
			dev_info(&dev->interface->dev, "sensor status");
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static long ft9201_ioctl_get_status(struct ft9201_device *dev, struct ft9201_status *device_status)
{
	unsigned char sensor_status;
	int errCode;

	if (device_status == NULL) {
		return -EFAULT;
	}

	errCode = ft9201_get_sensor_int_port_states(dev, &sensor_status);
	if (errCode < 0) {
		return errCode;
	}

	device_status->initialized = dev->device_status.initialized;
	device_status->sui_version = 0;
	device_status->sensor_mcu_state = sensor_status;
	device_status->chip_variant = 1;
	device_status->sensor_width = FT9201_IMG_WIDTH;
	device_status->sensor_height = FT9201_IMG_HEIGHT;
	device_status->afe_chip_id = PRODUCT_ID;
	device_status->fw_version = 0;
	device_status->agc_version = 0;

	return 0;
}

static DECLARE_WAIT_QUEUE_HEAD(ft9201_wq);

static int has_data_remaining(struct ft9201_device *dev)
{
	return dev->img_in_copied < dev->img_in_filled;
}

static ssize_t send_read_data(struct ft9201_device *dev, char __user *buf, size_t count)
{
	size_t remaining = dev->img_in_filled - dev->img_in_copied;
	size_t to_copy = remaining;
	if (to_copy > count) {
		to_copy = count;
	}
	dev_info(&dev->interface->dev, "Copied: %lu, to_copy: %lu, full_size: %lu", dev->img_in_copied, to_copy, dev->img_in_size);

	if (dev->img_in_copied + to_copy > dev->img_in_size) {
		return -EINVAL;
	}

	if (copy_to_user(buf, dev->read_img_data + dev->img_in_copied, to_copy)) {
		return -EFAULT;
	}
	dev->img_in_copied += to_copy;
	return (ssize_t)to_copy;
}

static ssize_t ft9201_read(struct file *fp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct ft9201_device *dev = fp->private_data;
	unsigned long loop_timeout = msecs_to_jiffies(100);
	int ret = 0;
	unsigned char data_ready;
	bool finger_present;

	if (dev == NULL) {
		pr_err("device is null\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev->io_mutex);
	if (ret < 0) {
		pr_info("Interrupted while waiting on IO mutex");
		return ret;
	}

	if (dev->disconnected) {
		ret = -ENODEV;
		goto exit;
	}

	while (true) {
		if (has_data_remaining(dev)) {
			ret = send_read_data(dev, buf, count);
			break;
		}

		ret = wait_event_interruptible_timeout(ft9201_wq, 0, loop_timeout);
		if (ret == -ERESTARTSYS) {
			/* We were interrupted by a signal */
			ret = 0;
			break;
		}

		/* Poll the sensor until it reports data ready. */
		ret = ft9201_get_sensor_int_port_states(dev, &data_ready);
		if (ret < 0) {
			break;
		}

		if (data_ready == 0x00) {
			continue;
		}

		/* Mirror the Windows sequence exactly: read the 32-byte info block
		 * (finger flag is its first four bytes), arm the next scan, then
		 * re-read the 4-byte finger status before capturing the image. */
		{
			unsigned char info_block[32];

			ret = ft9201_read_info_block(dev, info_block);
			if (ret < 0) {
				break;
			}
		}

		ret = ft9201_arm_capture(dev);
		if (ret < 0) {
			break;
		}

		/* A finger must be present for a valid capture. */
		ret = ft9201_finger_present(dev, &finger_present);
		if (ret < 0) {
			break;
		}

		if (!finger_present) {
			continue;
		}

		ret = ft9201_read_image(dev);
		if (ret < 0) {
			break;
		}
	}

exit:
	mutex_unlock(&dev->io_mutex);
	return ret;
}

static void ft9201_delete(struct kref *kref)
{
	struct ft9201_device *dev = to_ft9201_dev(kref);

	usb_put_intf(dev->interface);
	usb_put_dev(dev->udev);
	if (dev->read_img_data != NULL) {
		kfree(dev->read_img_data);
		dev->read_img_data = NULL;
	}
	kfree(dev);
}

static int ft9201_probe(struct usb_interface *intf, const struct usb_device_id *id) {
	struct usb_device *udev = interface_to_usbdev(intf);
	struct ft9201_device *dev;
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;

	int retval;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}

	kref_init(&dev->kref);
	mutex_init(&dev->io_mutex);

	dev->udev = usb_get_dev(udev);
	dev->interface = usb_get_intf(intf);

	/* use only the first bulk-in and bulk-out endpoints */
	retval = usb_find_common_endpoints(intf->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
	if (retval) {
		dev_err(&intf->dev, "Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	dev->bulk_in_endpointAddr = bulk_in->bEndpointAddress;

	/* save our data pointer in this interface device */
	usb_set_intfdata(intf, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(intf, &ft9201_class);
	if (retval) {
		/* something prevented us from registering this driver */
		dev_err(&intf->dev,
				"Not able to get a minor for this device.\n");
		usb_set_intfdata(intf, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&intf->dev, "USB fpreader device now attached to fpreader%d", intf->minor);

	retval = ft9201_initialize(dev);
	if (retval < 0) {
		dev_err(&dev->interface->dev, "Error initializing device: %d", retval);
	}

	return 0;

error:
	/* this frees allocated memory */
	kref_put(&dev->kref, ft9201_delete);

	return retval;
}

static void ft9201_disconnect(struct usb_interface *interface) {
	struct ft9201_device *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	if (dev == NULL) {
		return;
	}
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &ft9201_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->disconnected = 1;
	mutex_unlock(&dev->io_mutex);

	/* decrement our usage count */
	kref_put(&dev->kref, ft9201_delete);

	dev_info(&interface->dev, "USB device fpreader%d now disconnected", minor);
}

static int ft9201_suspend(struct usb_interface *intf, pm_message_t message) {
	pr_info("Suspend");

	return 0;
}

static int ft9201_resume(struct usb_interface *intf) {
	pr_info("Resume");

	return 0;
}

static struct usb_driver ft9201_driver = {
		.name = "ft9201",
		.probe = ft9201_probe,
		.disconnect = ft9201_disconnect,
		.suspend = ft9201_suspend,
		.resume = ft9201_resume,
		.id_table = ft9201_table,
};

module_usb_driver(ft9201_driver);