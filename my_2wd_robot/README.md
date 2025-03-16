# My 2WD Robot Project [Work In Progress]

This repository contains the ROS2 package for a **custom 2WD robot**, including its URDF model, launch files, visualization configurations, and custom scripts. The package is designed to work with ROS2 Humble and can be used to simulate and control the robot in a Gazebo or RViz environment.

![Robot Overview](path/to/your/image.png)

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Simulation](#simulation)
- [Control](#control)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Introduction
This project involves the design and control of a 2WD robot using ROS2 Humble and Gazebo. The robot is simulated, and various functionalities, including navigation and sensor integration, are implemented for testing and experimentation purposes.

## Features
- **Differential Drive:** A 2-wheel robot with a differential drive system.
- **Simulation:** A Gazebo environment setup for testing the robot's navigation and control.
- **ROS2 Integration:** Uses ROS2 Humble for communication and control.
- **Sensor Support:** Support for sensors like camera, LIDAR and Depth Camera. (expandable)
- **Customization:** Easily customizable for different use cases and environments.

## Project Structure
Here is an overview of the folder structure in the repository:


```plaintext
my_2wd_robot/
├── launch/
├── src/
├── urdf/
├── config/
├── worlds/
└── README.md
```
launch/: Contains ROS2 launch files for starting the robot simulation.
src/: Source code for robot control and sensor integration.
urdf/: Robot description files in URDF format.
config/: Configuration files for different ROS2 nodes.
worlds/: Custom Gazebo worlds for simulation.


## Prerequisites
- **Operating System:** Ubuntu 20.04 or later
- **ROS2 Distribution:** Humble
- **Gazebo:** Version 11 or later
- **Python 3.x**
- **C++ Compiler:** GCC for building ROS2 nodes


## Installation
1. Clone the Repository
```bash
mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/VedantC2307/MultiRobotProject.git
```
2. Install Dependencies Install ROS2 and other necessary dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

## Usage
### Launch Simulation in Gazebo
To start the robot simulation in Gazebo, use the launch file provided:
```bash
ros2 launch my_2wd_robot my_robot.launch.py
```

### Control the Robot
To control the robot using the keyboard or a predefined script:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
You can also use your own control scripts by placing them in the src/ directory.

### Visualize in RViz
Launch RViz to visualize sensor data and the robot's state:
```bash
ros2 launch my_2wd_robot view_robot.launch.py
```
### Simulation

The robot can be tested in a simulated Gazebo environment. You can modify the world files located in the worlds/ directory to create custom simulation scenarios.
Control

This project includes scripts for basic robot control using differential drive. You can extend this by adding additional control strategies and sensors.
Contributing
