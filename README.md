# FT9201 fingerprint reader Linux driver

Linux char-device driver for the FocalTech (Focal-systems.Corp) FT9201
fingerprint sensor, **USB ID `2808:9338`**.

The USB protocol was reverse-engineered from the official Windows driver
(v2.0.3.83) using USBPcap captures and verified against a live sensor through
this driver. See the protocol notes in `ft9201.c`.

## Status

- Captures raw 64x80 grayscale fingerprint images at `/dev/fpreader*`.
- Exposes init/status/power ioctls consumed by the companion `ft9201_util`
  utility.
- Not yet integrated with libfprint; it will not work with desktop login
  settings. It is intended for custom programs.

## Prerequisites

Verify your device is present:

```shell
lsusb
```

You should see `2808:9338 Focal-systems.Corp FT9201Fingerprint`.

## Build

```shell
make
```

This builds the kernel module (`ft9201.ko`) and the userspace utility
(`ft9201_util`).

## Install (with Secure Boot)

1. Generate a key pair:
   ```shell
   openssl req -new -x509 -newkey rsa:2048 -keyout MOK.priv -outform DER -out MOK.der -nodes -days 36500 -subj "/CN=Custom Kernel Module Signing/"
   ```
2. Sign the module:
   ```shell
   sudo /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 ./MOK.priv MOK.der ft9201.ko
   ```
3. Enroll the key:
   ```shell
   sudo mokutil --import MOK.der
   ```
4. Reboot and approve the key in the MOK manager.

Alternatively, disable Secure Boot.

## Load

Install and load the module:

```shell
sudo make install
sudo depmod -a
sudo modprobe ft9201
```

Or load it manually without installing:

```shell
sudo insmod ./ft9201.ko
```

When the sensor is present, the driver creates `/dev/fpreader0`.

## Usage

```shell
sudo ./ft9201_util /dev/fpreader0
```

Capture a raw image and convert it to PNG:

```shell
sudo cat /dev/fpreader0 > fingerprint.rawimg
convert -size 64x80 -depth 8 gray:./fingerprint.rawimg fingerprint.png
```

## License

GPL v2. See `LICENSE`.