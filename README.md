# CEC Linux Driver for SECO X86 Boards

Sources for SECO CEC Driver. Based on Linux CEC Framework.

Compatible with:

* UDOO X86 (all versions)

## Compile

Make sure you have Linux kernel headers >= 4.10.

```bash
# install dependencies (make, git, ecc..)
sudo apt install git build-essential linux-headers-`uname -r`

# clone repo
git clone https://github.com/ektor5/secocec

# go to dir
cd secocec

# compile
make
```

The module should appear as `seco-cec.ko`.

## Mount

This module depends on the Linux CEC Framework Module, and it needs to be loaded first.

```bash
modprobe cec
```

Note: this module conflicts with the official SMBus driver *i2c-i802* and
therefore it needs to be blacklisted. On Ubuntu should be already blacklisted.

```bash
echo "blacklist i2c-i802" > /etc/modprobe.d/i2c-i802.conf
```

Load the module:

```bash
insmod seco-cec.ko
```

Done!

## Use

To test the device from userspace (`/dev/cec0`), there are several programs
ready to use in [v4l-utils][v4l-utils] tools package (available to install via
apt):

* `cec-ctl`: An application to control cec devices
* `cec-compliance`: An application to verify remote CEC devices
* `cec-follower`: An application to emulate CEC followers

Check the corresponding man pages for info.

[v4l-utils]: https://git.linuxtv.org/v4l-utils.git/

## Debug

To compile the module with debug messages:

``` bash
make DEBUG=1
```

Feel free to open issues or mail me directly. Make sure to include *dmesg*,
*lsmod*, etc.. in the bug report.

## Credits

Author: Ettore Chimenti  
Thanks to: Hans Verkuil

Copyright (C) 2017, Seco Srl  
Copyright (C) 2017, Aidilab

