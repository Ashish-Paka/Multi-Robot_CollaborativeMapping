# Multi-Robot Collaborative Mapping and Exploration

## Introduction

This project presents a state-of-the-art multi-robot system for **collaborative exploration**, **mapping**, and **3D scene reconstruction** in complex, dynamic environments. Our framework integrates advanced sensor fusion, SLAM, decentralized coordination, and rigorous control theory. Each robot is equipped with high-precision LiDAR, 2D TOF cameras, and additional sensors that enable robust situational awareness and high-accuracy mapping. Beyond classical control techniques like PID, we incorporate modern methodologies—including Reinforcement Learning (RL) and swarm intelligence—to enable adaptive, intelligent behavior.

---

## System Overview

### Multi-Robot and Swarm Robotics

- **Collaborative Exploration & Mapping:**  
  Each robot autonomously constructs a local occupancy grid map using sensor data. These local maps are then registered and fused into a global map using iterative registration methods (similar to ICP) and pose graph optimization. This process reduces drift and ensures global consistency.

- **Decentralized Communication:**  
  Using a ROS2 multi-master configuration, robots exchange sensor data, local maps, and control signals in real time. This decentralized framework supports dynamic role assignments (e.g., leader/follower) and task allocation based on environmental conditions.

- **Formation Control:**  
  Formation control is achieved through consensus protocols and potential-field methods. Robots adjust their relative positions to maintain coordinated coverage of the workspace while avoiding collisions.

---

## Technical Methodologies

### Robotics, SLAM, and Control Systems

1. **Sensor Fusion & Perception**  
   - **Multi-Sensor Integration:** Sensor data from LiDAR, TOF cameras, and IMUs are fused using pre-computed transformation matrices. Data filtering techniques (e.g., voxel grid filtering) remove noise and outliers.  
   - **Perception Pipeline:** The system processes raw sensor data to generate reliable estimates of the environment. This includes associating sensor measurements with environmental features and computing probabilistic occupancy grids.

2. **SLAM and Graph Optimization**  
   - **Local Mapping:** Each robot builds an occupancy grid using Bayesian updates, where cell probabilities are refined based on new sensor readings.  
   - **Pose Graph Optimization:** The local pose graph is constructed by treating each robot pose as a node and sensor-based measurements as edges. An optimization algorithm (nonlinear least-squares, similar to methods used in g2o) minimizes the error in relative poses, reducing drift and aligning overlapping areas.

3. **PID and Advanced Motion Control**  
   - **PID Control:** Each robot’s motion is governed by a PID controller that minimizes the error between the desired and actual states. The controller considers proportional, integral, and derivative components to ensure smooth velocity and orientation tracking.  
   - **Path Planning & Smoothing:** Motion planning utilizes Hybrid-A* algorithms for obstacle-aware trajectory generation. Cubic spline interpolation is then applied to smooth the path, ensuring continuity and reducing mechanical stress on actuators.

4. **Reactive Obstacle Avoidance**  
   - **Potential Fields:** The system uses potential fields to compute repulsive forces from nearby obstacles, while simultaneously computing attractive forces toward navigation goals. This combined force vector modulates the velocity commands in real time, ensuring collision-free navigation.

### Software Architecture

- **ROS2-Based Middleware:**  
  A modular architecture where individual ROS2 nodes manage sensor acquisition, data processing, control, and inter-robot communication. Synchronized `tf` transforms maintain consistent coordinate frames across nodes, while simulation tools (Gazebo) and visualization (RViz) support debugging and performance assessment.

- **Data Processing Pipeline:**  
  Advanced SLAM modules integrate feature extraction and environmental modeling. Techniques from machine learning and deep learning are under exploration to further improve sensor data interpretation, especially under challenging conditions.

### Electronics and Embedded Systems

- **Sensor Integration and Calibration:**  
  High-speed sensor data is synchronized via precise calibration routines. Pre-computed transformation matrices align LiDAR, TOF, and inertial measurements into a common reference frame, ensuring accurate data fusion.

- **Real-Time Control and Power Management:**  
  Custom electronics manage high-frequency control loops and sensor data processing. Embedded controllers execute PID loops for motion control, while dedicated circuits handle battery monitoring, voltage regulation, and thermal management to maintain system reliability during extended missions.

### Mechanical Design and Structural Analysis

- **Rapid Prototyping and Finite Element Analysis (FEA):**  
  The robot chassis and components are designed with CAD/CAM tools and fabricated using 3D printing techniques. FEA is used to simulate stress distributions and validate the structural integrity under dynamic and static loads, ensuring durability during operation.

- **Composite Materials and Aerodynamics:**  
  Optimized designs incorporate lightweight composite materials to achieve a high strength-to-weight ratio. Aerodynamic profiling and strategic center-of-gravity placement enhance energy efficiency and maneuverability, particularly during high-speed or agile maneuvers.

---

## Detailed Framework Explanation

### 1. Concepts and Methods

#### Multi-Robot Systems & Swarm Robotics

- **Distributed SLAM:**  
  Each robot maintains a local occupancy grid updated with Bayesian probability methods, considering both prior and new sensor measurements. Local maps are aligned through an iterative registration process and fused via a global pose graph optimization strategy. This distributed SLAM approach minimizes drift and enhances overall map accuracy.

- **Decentralized Coordination:**  
  Communication is managed through ROS2 topics, with role assignments (e.g., leader/follower) determined by auction-based or consensus-based methods. These strategies enable efficient task division and coordinated navigation without a centralized controller.

#### Robotics, SLAM, and Control Systems

- **Sensor Fusion & SLAM:**  
  Raw sensor data is pre-processed to filter noise, then aligned using transformation matrices. Scan matching techniques improve the alignment of consecutive frames, while graph-based optimization ensures that the final global map is consistent and accurate.

- **Control Strategies:**  
  PID controllers form the backbone of the control system, continuously adjusting the robot’s velocity and orientation to minimize tracking errors. Reactive behaviors, such as obstacle avoidance, are integrated via potential field methods, allowing the system to adapt quickly to dynamic changes.

### 2. Step-by-Step Working of the Framework

#### 2.1 Initialization and Calibration

1. **ROS2 Node Deployment:**  
   - Robots start by launching ROS2 nodes dedicated to sensor acquisition, processing, and communication.  
   - Nodes register on a multi-master network, allowing decentralized coordination and real-time data sharing.

2. **Sensor Calibration:**  
   - Static and dynamic calibration processes align LiDAR, TOF, and IMU sensors into a unified coordinate frame using transformation matrices (comprising rotation and translation components).

3. **Role Assignment:**  
   - Based on sensor quality and current mapping performance, dynamic role assignment algorithms (e.g., auction-based methods) determine which robot acts as a leader and which act as supporting units.

#### 2.2 Local Sensing, Perception, and Mapping

1. **Data Acquisition:**  
   - Continuous sensor data (point clouds, depth maps, inertial data) is sampled at high frequencies. Noise reduction techniques, such as voxel grid filtering, are applied to enhance data quality.

2. **Local Mapping:**  
   - Robots update occupancy grids by integrating new sensor measurements with prior probabilities, using Bayesian update methods.  
   - Scan matching aligns successive sensor frames, and a local pose graph maintains relative transformations between scans.

#### 2.3 Communication and Global Map Fusion

1. **Inter-Robot Data Exchange:**  
   - Local maps, robot poses, and metadata are published on ROS2 topics. Precise timestamp synchronization ensures data consistency across the network.

2. **Global Map Construction:**  
   - A dedicated map-merge module aggregates local maps into a single, coherent global representation.  
   - The fusion process involves transforming local maps into a common reference frame, followed by global pose graph optimization to resolve overlaps and misalignments.

#### 2.4 Motion Control and Navigation

1. **Trajectory Planning:**  
   - Using algorithms like Hybrid-A*, robots compute collision-free paths in real time.  
   - Cubic spline interpolation is applied to smooth trajectories, ensuring continuity and reducing mechanical wear.

2. **Reactive Control:**  
   - PID controllers adjust the robot’s speed and direction based on real-time error feedback.  
   - Reactive obstacle avoidance combines outputs from the PID controller with potential field calculations to modify trajectories on the fly.

#### 2.5 Adaptive Behavior and Map Refinement

1. **Stochastic Re-Planning:**  
   - In complex or highly cluttered environments, robots execute random rotational maneuvers in addition to deterministic controls, helping them escape local minima.

2. **Iterative Map Update:**  
   - As new sensor data arrives, both local and global pose graphs are continuously re-optimized, progressively enhancing the overall map accuracy.

---

## Future Work

### 3. Future Work and Reinforcement Learning Integration

#### 3.1 Reinforcement Learning (RL) for Adaptive Navigation and Task Allocation

- **Action Space Expansion:**  
  The control framework will be extended to include more nuanced actions such as adaptive speed control, dynamic goal switching, and enhanced environmental interactions. This will allow robots to learn complex behaviors tailored to varying mission demands.

- **Reward Function Design:**  
  A comprehensive reward structure will be developed to balance exploration (maximizing environmental coverage) and exploitation (minimizing collisions and energy consumption). Rewards will penalize unsafe behaviors while encouraging efficient, high-coverage navigation.

- **RL Algorithms and Training:**  
  Multi-Agent Proximal Policy Optimization (MAPPO) or similar algorithms will be used to train decentralized policies in simulation environments like Gazebo. These policies will be refined through iterative training and validated with performance metrics before being transferred to physical robots.

- **Policy Transfer and Real-World Adaptation:**  
  Strategies such as domain randomization and online fine-tuning will be employed to transfer policies from simulation to real-world hardware, ensuring robustness under diverse environmental conditions.

#### 3.2 Expanded Swarm, Perception, and Physical Implementation

- **Advanced Coordination:**  
  Future research will enhance consensus algorithms for dynamic formations and distributed task allocation. This will improve multi-robot cooperation in complex tasks such as cooperative object manipulation and large-area mapping.

- **Enhanced Sensor Fusion:**  
  Incorporating deep learning techniques for real-time object detection and semantic segmentation will improve environmental understanding. This enhancement will allow the system to better identify obstacles and features, leading to more accurate SLAM.

- **Physical Implementation and Integration:**  
  - **Hardware Upgrades:** Future work will focus on the integration of custom PCBs, more robust power management systems, and advanced sensor suites to improve the overall reliability and performance of the robots in real-world conditions.  
  - **Field Trials:** Extensive physical deployments in varied environments (e.g., urban, industrial, and natural terrains) will be conducted to validate and refine the system. These trials will focus on real-time mapping accuracy, robustness against environmental disturbances, and energy efficiency during prolonged operations.  
  - **Human-Robot Collaboration:** Developing intuitive user interfaces and robust communication protocols will facilitate seamless interaction between human operators and autonomous systems. This integration is essential for applications such as disaster response, industrial inspection, and environmental monitoring.

---

**Summary**  
This README provides a comprehensive, technically detailed explanation of our multi-robot collaborative mapping framework. It covers the key concepts, the step-by-step working of the system, and outlines future directions including reinforcement learning integration and physical implementation. Our goal is to deliver a robust, scalable, and cutting-edge solution for autonomous multi-robot exploration and mapping, bridging advanced theory with practical, real-world applications.



Now how to run:
## Prerequisites
- ROS2 installed (tested with ROS2 Humble/Foxy)
- TurtleBot3 simulation dependencies installed
- A properly configured ROS2 workspace (`<ros2_ws>`)

## Installation

1. Navigate to your workspace source directory:
    ```bash
    cd <ros2_ws>/src
    ```

2. Clone the repository:
    ```bash
    git clone https://github.com/VedantC2307/MAE598-MultiRobotSystemsProject.git
    ```

3. Build the workspace:
    ```bash
    cd <ros2_ws>
    colcon build --symlink-install
    ```
    If you encounter any errors, try building again:
    ```bash
    colcon build --symlink-install
    ```

## Running the Package

### Step 1: Launch the Simulation
Open a terminal and run the following commands to launch the multi-robot simulation with SLAM:
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
source <ros2_ws>/install/setup.bash
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_gmapping:=True
```
### Step 2: Start Random Walk Exploration
In a second terminal, source the workspace and start the random walk exploration node for robot 1:
```bash
source <ros2_ws>/install/setup.bash
ros2 run multi_robot_explore multi_robot_random_walk_robot1 --ros-args --params-file ./src/multi_robot_explore/config/robot_params.yaml
```
Open third terminal, source the workspace and start the random walk exploration node for robot 2:
```bash
source <ros2_ws>/install/setup.bash
ros2 run multi_robot_explore multi_robot_random_walk_robot2 --ros-args --params-file ./src/multi_robot_explore/config/robot_params.yaml
```

### Step 3: Launch the Map Merge Node
In a forth terminal, launch the map merge node to combine the maps from multiple robots:
```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

### Step 4: Visualize in RViz

In a fivth terminal, launch RViz for visualization:
```bash
rviz2 -d <ros2_ws>/src/m-explore-ros2/map_merge/launch/map_merge.rviz
```


# Please find the attached reports and documents in the docs folder. Thanks to my partners Vedant C and Akshay M.