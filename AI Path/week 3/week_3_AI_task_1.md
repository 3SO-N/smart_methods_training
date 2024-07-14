# Week 3 AI Task 1: Controlling the Robot Arm

## Task Overview
In this task, you will control a robot arm using ROS (Robot Operating System). The task is divided into two main parts:
1. Controlling the robot arm by `joint_state_publisher`.
2. Controlling the robot arm using Moveit and kinematics.

The steps below will guide you through the process of setting up and controlling the robot arm.

## Part 1: Controlling the Robot Arm by `joint_state_publisher`

### Step 1: Add the `arduino_robot_arm` package to the `src` folder
Open a terminal and run the following commands:
```sh
$ cd ~/catkin_ws/src
$ sudo apt install git
$ git clone https://github.com/smart-methods/arduino_robot_arm 
```

### Step 2: Install all the dependencies
Navigate to your workspace and install the necessary dependencies:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ sudo apt-get install ros-noetic-moveit
$ sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
$ sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
```

### Step 3: Compile the package
```sh
$ catkin_make
```

### Step 4: Launch the `check_motors` launch file
```sh
$ roslaunch robot_arm_pkg check_motors.launch
```

This will launch the simulation of the robot arm, and you should be able to see it move according to the joint states.

## Part 2: Controlling the Robot Arm using Moveit and Kinematics

### Step 1: Launch the Moveit demo
Moveit provides a way to plan and execute motions for the robot arm. To launch the Moveit demo, run:
```sh
$ roslaunch moveit_pkg demo.launch
```

### Step 2: Using RViz with Moveit
Once the demo is launched, you can use RViz, a 3D visualization tool for ROS, to interact with the robot arm. You can plan motions and visualize the planned paths in RViz. 

### Step 3: Understanding Kinematics
Kinematics allows you to control the robot arm by specifying the end effector's position and orientation. Moveit uses inverse kinematics to compute the required joint states to reach a specific end effector position. You can experiment with different end effector goals in RViz and see how the robot arm responds.

## File Structure
To help you understand the directory structure, here is an overview of the important directories and files:
```sh
username@ubu20:~$ cd ~/catkin_ws/
username@ubu20:~/catkin_ws$ ls
build  devel  src
username@ubu20:~/catkin_ws$ cd src
username@ubu20:~/catkin_ws/src$ ls
arduino_robot_arm  CMakeLists.txt
username@ubu20:~/catkin_ws/src$ cd arduino_robot_arm/
username@ubu20:~/catkin_ws/src/arduino_robot_arm$ ls
arduino_code  circuit.png  moveit_pkg  positions.png  README.md  robot_arm_pkg
username@ubu20:~/catkin_ws/src/arduino_robot_arm$ cd robot_arm_pkg/
username@ubu20:~/catkin_ws/src/arduino_robot_arm/robot_arm_pkg$ ls
CMakeLists.txt  config  launch  meshes  package.xml  scripts  urdf
username@ubu20:~/catkin_ws/src/arduino_robot_arm/robot_arm_pkg$ cd ..
username@ubu20:~/catkin_ws/src/arduino_robot_arm$ cd moveit_pkg/
username@ubu20:~/catkin_ws/src/arduino_robot_arm/moveit_pkg$ ls
CMakeLists.txt  config  launch  package.xml  scripts  src
```

By following the steps above, you should be able to control the robot arm using both `joint_state_publisher` and Moveit.
