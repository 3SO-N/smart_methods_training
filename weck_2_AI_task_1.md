
# Week 2 AI Task 1: Install ROS Noetic and ROS Foxy

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Installing ROS Noetic](#installing-ros-noetic)
4. [Installing ROS Foxy](#installing-ros-foxy)
5. [Verification](#verification)
6. [Conclusion](#conclusion)

## Introduction
This document provides a step-by-step guide to install ROS Noetic and ROS Foxy on your system. ROS (Robot Operating System) is a flexible framework for writing robot software. Noetic is the latest ROS 1 release, while Foxy is one of the recent ROS 2 releases.

## Prerequisites
- A computer running Ubuntu 20.04 (for ROS Noetic)
- A computer running Ubuntu 20.04 or Ubuntu 22.04 (for ROS Foxy)
- A user account with sudo privileges
- Basic knowledge of using the terminal

## Installing ROS Noetic

### Step 1: Set up your sources.list
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Step 2: Set up your keys
```sh
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Step 3: Update the package index
```sh
sudo apt update
```

### Step 4: Install ROS Noetic
```sh
sudo apt install ros-noetic-desktop-full
```

### Step 5: Initialize rosdep
```sh
sudo rosdep init
rosdep update
```

### Step 6: Environment setup
Add the following line to your `.bashrc` file:
```sh
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7: Install dependencies for building packages
```sh
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## Installing ROS Foxy

### Step 1: Set up your sources.list
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### Step 2: Set up your keys
```sh
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Step 3: Update the package index
```sh
sudo apt update
```

### Step 4: Install ROS Foxy
```sh
sudo apt install ros-foxy-desktop
```

### Step 5: Environment setup
Add the following line to your `.bashrc` file:
```sh
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Install dependencies for building packages
```sh
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### Step 7: Initialize rosdep
```sh
sudo rosdep init
rosdep update
```

## Verification
To verify that both ROS Noetic and ROS Foxy have been installed correctly, open a new terminal and run the following commands:

For ROS Noetic:
```sh
source /opt/ros/noetic/setup.bash
roscore
```

For ROS Foxy:
```sh
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

If both commands run without errors, the installations are successful.

## Conclusion
This document covered the steps to install ROS Noetic and ROS Foxy on an Ubuntu system. Following these instructions ensures a smooth installation process. If you encounter any issues, refer to the ROS installation guides or seek help from the ROS community.
