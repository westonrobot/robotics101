---
layout: page
title: Ubuntu Installation
permalink: /lab_sessions/lab1/prelab
parent: Lab 1
nav_order: 1
---

## Install Ubuntu 18.04

> ***NOTICE:*** Before proceeding with any ubuntu installation (especially when dual booting), it
> is highly recommended that you backup any important data you have on the computer.
> We will not be responsible for any loss of data and cannot provide assistance with
> any data recovery attempts.

### 1. Preparation

You need to set up a bootable USB drive for Ubuntu installation. You need the following:

- A USB drive with a minimum capacity of 4GB
- [Ubuntu 18.04 Desktop Image](https://releases.ubuntu.com/18.04)
- Windows/Linux application for creating a bootable USB drive: Rufus or Etcher
    - [Rufus](https://rufus.ie/en/)
    - [Etcher](https://www.balena.io/etcher)

**IMPORTANT** : You will need to format the USB drive, so backup your data on the USB drive first.

After creation, verify you can boot from your USB drive. Usually you can choose which disk to boot from by pressing F2 or F10 or ESC at the boot screen. Search the Internet for information specific to your computer brand and model. If unable to boot from USB, try disabling secure boot/fast boot in BIOS configuration.

### 2. Disk Partition

Suggested partitioning Size:

* **/**: root partition, ext4 format, >=25GB 
* **/home**: ext4 format, allocate it rest of the free space (as much as possible), this is the place where you put most of your work files 
* **swap**: swap area, 4~8G should be sufficient for most computers

### 3. Installation

You mostly only need to follow the Ubuntu installation guide and provide information such as user name, computer name, password etc.

You're encouraged to refer to online materials for detailed guide for installing Ubuntu:
- [Dual Boot](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/)
- [Bootable USB drive](https://itsfoss.com/intsall-ubuntu-on-usb/)
- [VM Oracle VirtualBox](https://brb.nci.nih.gov/seqtools/installUbuntu.html#:~:text=Select%20your%20new%20virtual%20machine,this%20page%20for%20more%20information.)

**NOTE**: Using virtual machine is not recommended for this course. You may encounter issues related to hardware access.