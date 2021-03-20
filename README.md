[![Actions Status](https://github.com/raetro-sigs/tools-rbfloader/workflows/Build/badge.svg)](https://github.com/raetro-sigs/tools-rbfloader/actions)
[![GitHub release](https://img.shields.io/github/release/raetro-sigs/tools-rbfloader.svg)](https://github.com/raetro-sigs/tools-rbfloader/releases)
[![license](https://img.shields.io/github/license/raetro-sigs/tools-rbfloader.svg)](https://github.com/raetro-sigs/tools-rbfloader/blob/master/LICENSE)

# FPGA RBF Loader for ARM Linux

Software to configure an **FPGA** using **JTAG** via the GPIO pins of an ARM SBC. It works under Linux focused on the **Raspberry Pi** and the **Altera Cyclone FPGA without HPS** family, but should be easily portable to other platforms.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Building and Installing

Installation is done from the git repository as follows:

```bash
sudo apt install build-essential cmake git
git clone https://github.com/raetro-sigs/tools-rbfloader
cd RBFLoader
mkdir build && cd build
cmake ..
cmake --build .
cpack
sudo dpkg --install *.deb
```

### JTAG Cable Configuration

The GPIO pins are set in `/etc/rbfloader.conf`, along with any other configuration values that may come in the future.

Here’s an example of the default pin configuration between a **Raspberry Pi Zero** and a **Trenz CYC1000**

| Key  | GPIO    | Description      |  FPGA GPIO    |
| ---- | ------- | ---------------- |  ------------ |
| TMS  | 04      | Test Mode Select |  7            |
| TCK  | 17      | Test Clock       |  11           |
| TDI  | 22      | Test Data In     |  15           |
| TDO  | 27      | Test Data Out    |  13           |

### Optional Configuration

| Key       | Value           | Description                                                  |
| --------- | --------------- | ------------------------------------------------------------ |
| CYCLONE_2 | false <default> | Skips the first 44 characters of the RBF (Cyclone II Header) |

## Command Line Usage

You can upload a raw bit file (rbf) using the RBFLoader in the terminal by typing the following:

```bash
sudo rbfloader <my-raw-bit-file.rbf>
```

### Output Example

```bash
sudo rbfloader msx_cyc1000.rbf

Uploading msx_cyc1000.rbf...
Device ID Code: 34549981
  rev: 0
  family: 2
  size: 243
  manuf: 110
  onebit: 1
RBF size: 718569 bytes
******************************** Done.
Total Uploaded:  718569 bytes
Time Elepsed: 1665ms
FPGA Configuration completed.
```

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Community, discussion, contribution, and support

Aside from contributing code, documentation and feature suggestions you can also engage via the following channels:

- [Rætro Discord](https://discord.gg/YDdmtwh)
	- User related channels `#tools-r-us`.
- [Telegram Group](https://t.me/CYC1000)


## Credits and acknowledgment

This software is powered by Open-Source Software and reused codes from other projects. A great thanks to their efforts!

A special thanks to Victor Trucco for the [**STM32 Firmware**](https://gitlab.com/victor.trucco/Multicore/-/tree/master/System/STM32) and Gordon Henderson for the amazing **wiringPi** library.

## Legal Notices

This project is licensed under the [BSD 3-Clause "New" or "Revised" License](https://spdx.org/licenses/BSD-3-Clause.html).

wiringPi is licensed under the [GNU General Public License v3.0 or later](https://spdx.org/licenses/GPL-3.0-or-later.html).

The authors, contributors or any of the maintainers of this project are in no way associated with or endorsed by ARM®, Intel®, Altera®, Raspberry Pi® or any other company not implicit indicated. All other brands or product names are the property of their respective holders.
