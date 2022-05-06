---
layout: page
title: PreLab 1
permalink: /lab_notes/lab1/lab
parent: Lab 1
nav_order: 1
---

## 1. Installation of ROS Melodic

### 1.1 Setup of sources.list

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

### 1.2 Setting up keys
    sudo apt-get install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

### 1.3 Installation
For this module, we will install the full ROS Melodic 

    sudo apt-get update
    sudo apt-get install ros-melodic-desktop-full

### Note
- apt-get 