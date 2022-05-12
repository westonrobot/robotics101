---
layout: page
title: Lab 1 - Linux basics & ROS installation
permalink: /lab_sessions/lab1
parent: Lab Sessions
nav_order: 1
---

# Lab1 (4%)

## Prelab (1%)

Before coming to the first lab, you are required to:

1. Form your team of 5
2. Install Ubuntu 18.04 on your computer

**Readings**

* [Notes on Ubuntu Installation]({{ site.baseurl }}{% link lab_sessions/lab1/prelab.md %})

## What You Will Need
* Your computer with ubuntu installed

## Learning Outcomes

By the end of Lab 1, you will have learnt:

1. Basic Linux bash commands  
2. Ubuntu's package manager (**apt**)  
3. Understand system variables (**.bashrc**)  
4. Install ROS Melodic
5. Familiarize with your limo

----
### Installing ROS Melodic

1. Setup sources.list  
    To install ROS, we will have to prepare your computer to accept software from packages.ros.org. To do that we have to add the ROS apt repository to our system's apt repository source index.
    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
    Command breakdown:
    * sudo - This command temporarily gives you administrator privileges to run certain commands.

2. Setup apt key  
   To get any software from a apt server, we have to get the key to that apt server.
   ```bash
   sudo apt install curl # if you haven't already installed curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```
   Command breakdown:
   * apt - apt is ubuntu's package manager used to get/remove software from apt servers.

3. Installing ros  
   ```bash
    sudo apt update
    sudo apt install ros-melodic-desktop-full
   ```
   Command breakdown:
   * apt update - This fetches any available updates from all apt servers. This does **not** install any updates, but rather just informs our system that a update is available.
   * apt install - This attempts to install a specified package, this installs latest package version available (ROS melodic in this case).

4. Environment setup  
    In order to use our newly installed ROS, we have to somehow inform our terminal where to find this new package. In linux, we use something called environment variables to do this.
    ```bash
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    Command breakdown:
    * echo - This makes bash repeat whatever you tell it to ("source /opt/ros/melodic/setup.bash" in this case)
    * \>> - This tells bash to place the text into a file (~/.bashrc in this case).
    * source - This essentially runs a file containing bash commands.
    * ~/.bashrc - This is a special file presnet in the home directory that contains bash commands that is runned once every time you open a new terminal.

5. ROS dependencies  
   There are certain dependencies packages that ROS needs, that we have to apt install. These packages are...
    * python-rosdep
    * python-rosinstall
    * python-rosinstall-generator
    * python-wstool
    * build-essential
    Using the information from step 3, install these dependencies. After installing these pacakges, run
    ```bash
    sudo rosdep init
    rosdep update
    ```

6. Verify installation  
   Now we can proceed with verifying our new ROS installation, by running
   ```bash
   roscore
   ```
   If there are no errors, congratulations on installing ROS successfully.

### Getting to know your LIMO
[LIMO User manual and start guide](https://github.com/agilexrobotics/limo-doc/blob/master/Limo%20user%20manual(EN).md)