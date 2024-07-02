# Week 2 AI Task 3: Use ROS1 Bridge to Print Topic from ROS1 to ROS2

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Setting Up the ROS1 Bridge](#setting-up-the-ros1-bridge)
4. [Printing a Topic from ROS1 to ROS2](#printing-a-topic-from-ros1-to-ros2)
5. [Conclusion](#conclusion)

## Introduction
This document provides step-by-step instructions to use the ROS1 bridge to print a topic from ROS1 (Noetic) to ROS2 (Foxy). The `ros1_bridge` package allows for communication between ROS1 and ROS2 systems, making it possible to share data between the two.

## Prerequisites
- ROS Noetic installed on Ubuntu 20.04
- ROS2 Foxy installed on Ubuntu 20.04 or Ubuntu 22.04
- A user account with sudo privileges
- `ros1_bridge` package installed

## Setting Up the ROS1 Bridge

### Step 1: Install Dependencies
Make sure you have sourced both ROS1 and ROS2 in your `.bashrc` file:
```sh
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
```

### Step 2: Install `ros1_bridge`
```sh
sudo apt update
sudo apt install ros-foxy-ros1-bridge
```

### Step 3: Build the `ros1_bridge` Package
If you are using a workspace, make sure to build the bridge from source:
```sh
mkdir -p ~/ros1_bridge_ws/src
cd ~/ros1_bridge_ws/src
git clone https://github.com/ros2/ros1_bridge.git
cd ~/ros1_bridge_ws
colcon build --packages-select ros1_bridge --cmake-force-configure
source install/local_setup.bash
```

## Printing a Topic from ROS1 to ROS2

### Step 1: Start ROS1 Master
Open a terminal and start the ROS1 master:
```sh
roscore
```

### Step 2: Start ROS2 Daemon
Open another terminal and start the ROS2 daemon:
```sh
ros2 daemon start
```

### Step 3: Run the ROS1 Bridge
In a new terminal, run the `ros1_bridge`:
```sh
ros2 run ros1_bridge dynamic_bridge
```

### Step 4: Publish a Topic in ROS1
Open a new terminal and run:
```sh
rosrun turtlesim turtlesim_node
```
In another terminal, run:
```sh
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.57" -r 1
```

### Step 5: Subscribe to the Topic in ROS2
Open a new terminal and run:
```sh
ros2 topic echo /turtle1/cmd_vel
```

You should see the messages published in ROS1 being echoed in ROS2.

## Conclusion
This document covered the steps to use the `ros1_bridge` to print a topic from ROS1 to ROS2. By following these instructions, you can effectively bridge communication between ROS1 and ROS2 systems, allowing for seamless data sharing.
