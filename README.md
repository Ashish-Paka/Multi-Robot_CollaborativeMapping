# Multi-Robot Collaborative Mapping and Exploration

## Introduction

This project presents a state-of-the-art multi-robot system for collaborative exploration, mapping, and 3D scene reconstruction in complex, dynamic environments. Leveraging advanced sensor fusion, SLAM, decentralized coordination, and control theory, our framework integrates heterogeneous platforms—each equipped with high-precision LiDAR, 2D TOF cameras, and complementary sensors—to produce robust situational awareness and high-accuracy mapping. In addition to classical robotics techniques (e.g., PID control), we incorporate modern methodologies including Reinforcement Learning (RL) and advanced swarm intelligence to enable adaptive, intelligent behavior.

---

## System Overview

### Multi-Robot and Swarm Robotics

- **Collaborative Exploration & Mapping:**  
  The system employs a distributed SLAM framework where each robot autonomously constructs a local occupancy grid map using sensor data and later fuses it into a global map. This is achieved through iterative registration (using ICP) and graph optimization (using frameworks like g2o) to minimize drift and error.

- **Decentralized Communication:**  
  ROS2 multi-master configuration facilitates real-time message exchange and data synchronization. The decentralized approach enables robust role assignment (leader/follower) and task allocation through auction-based or consensus algorithms.

- **Formation Control:**  
  Formation and dynamic task allocation are managed via consensus protocols and potential field methods, ensuring coordinated coverage of the workspace.

---

## Technical Methodologies

### Robotics, SLAM, and Control Systems

- **Sensor Fusion & Perception:**  
  Multiple sensor streams (LiDAR, TOF, IMUs) are fused using transformation matrices (from calibration) and filtering algorithms (e.g., voxel grid filtering for point clouds). The underlying model transforms raw sensor measurements \( z \) to an estimated state \( \hat{x} \) using:
  \[
  \hat{x} = f(x, u) + w, \quad z = h(\hat{x}) + v
  \]
  where \( f \) is the motion model, \( h \) is the measurement model, and \( w, v \) denote process and measurement noise.

- **SLAM and Graph Optimization:**  
  Local maps are generated using Bayesian occupancy grids and refined via scan matching. The pose graph is defined by:
  \[
  \min_{X} \sum_{i,j} \| z_{ij} - h(x_i, x_j) \|^2_{\Sigma_{ij}^{-1}}
  \]
  where \( x_i \) and \( x_j \) are poses, \( z_{ij} \) the measured relative transformation, and \( \Sigma_{ij} \) the covariance. Optimization methods (e.g., g2o) solve this nonlinear least-squares problem.

- **PID and Advanced Motion Control:**  
  The motion control law is implemented using PID controllers. The control input is computed as:
  \[
  u(t) = K_p e(t) + K_i \int_0^t e(\tau) \, d\tau + K_d \frac{de(t)}{dt}
  \]
  where \( e(t) \) is the error between desired and current state. Cubic spline interpolation and Hybrid-A* planning further smooth the path in complex environments.

- **Reactive Obstacle Avoidance:**  
  Obstacles are avoided using potential fields where the repulsive force from an obstacle at distance \( d_i \) is given by:
  \[
  F_{\text{repulsive}} = \sum_{i=1}^{N} \frac{K_r}{d_i^2} \hat{d}_i
  \]
  This force is combined with the attractive force towards the goal, ensuring smooth, collision-free trajectories.

### Software Architecture

- **ROS2-Based Middleware:**  
  Modular ROS2 nodes handle sensor acquisition, processing, and inter-robot communication. Synchronized tf transforms maintain consistency between various coordinate frames, while simulation in Gazebo and visualization in RViz facilitate testing and debugging.

- **Advanced Data Processing:**  
  SLAM modules integrate ML/DL techniques for enhanced feature extraction and environment understanding, improving mapping robustness under diverse conditions.

### Electronics and Embedded Systems

- **Sensor Integration and Calibration:**  
  Rigorous calibration routines align sensor modalities into a common reference frame, ensuring that LiDAR, TOF, and inertial measurements are accurately fused.

- **Power Management and Control Loops:**  
  Custom electronics manage high-speed sensor data and control loops. Real-time battery monitoring, voltage regulation, and thermal management systems ensure reliable operation during prolonged missions.

### Mechanical Design and Structural Analysis

- **Rapid Prototyping and FEA:**  
  The robot chassis is designed using CAD/CAM tools and fabricated through 3D printing. Finite Element Analysis (FEA) simulates stress and strain distributions, ensuring structural integrity under dynamic loads.
  
- **Composite Materials and Aerodynamics:**  
  Design optimizations include material selection (e.g., composites for high strength-to-weight ratios) and aerodynamic profiling to improve energy efficiency and stability during high-speed maneuvers.

---

## Detailed Framework Explanation

### 1. Concepts and Methods

#### Multi-Robot Systems & Swarm Robotics

- **Distributed SLAM:**  
  Each robot constructs local maps via Bayesian updates:
  \[
  P(m|z_{1:t}, x_{1:t}) \propto P(z_t|m, x_t) P(m|z_{1:t-1}, x_{1:t-1})
  \]
  and fuses them into a global map using iterative closest point (ICP) and pose graph optimization.

- **Decentralized Coordination:**  
  Communication is managed using ROS2 topics, with role allocation determined by auction-based methods. Consensus algorithms ensure formation control and efficient task division.

#### Robotics, SLAM, and Control Systems

- **Sensor Fusion & SLAM:**  
  Data from LiDAR and TOF cameras are pre-processed and aligned using transformation matrices. SLAM employs both iterative scan matching and global optimization to reduce drift.
  
- **Control Theory:**  
  The control system leverages PID control along with potential field methods. The error dynamics are defined as:
  \[
  \dot{e}(t) = \dot{x}_{\text{desired}}(t) - \dot{x}(t)
  \]
  ensuring that the control input minimizes error while maintaining smooth trajectories.

### 2. Step-by-Step Working of the Framework

#### 2.1 Initialization and Calibration

- **ROS2 Node Deployment:**  
  Robots launch ROS2 nodes for sensor data capture and control. Nodes register with a multi-master system for decentralized coordination.
  
- **Sensor Calibration:**  
  Both static and dynamic calibration align LiDAR, TOF, and IMU measurements using pre-computed transformation matrices:
  \[
  T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}
  \]
  where \( R \) is the rotation matrix and \( t \) is the translation vector.

- **Role Assignment:**  
  Dynamic role allocation via auction-based algorithms assigns leader and follower roles based on sensor quality and current mapping performance.

#### 2.2 Local Sensing, Perception, and Mapping

- **Data Acquisition:**  
  Continuous sensor streams are filtered (e.g., using voxel grid filtering) to generate noise-reduced point clouds and depth maps.

- **Local Mapping:**  
  Local occupancy grids are built using Bayesian probability:
  \[
  \text{logit}(p) = \text{logit}(p_{\text{prior}}) + \sum_{i} \text{logit}(p(z_i|m))
  \]
  ICP aligns successive scans, while graph optimization minimizes global errors.

#### 2.3 Communication and Global Map Fusion

- **Inter-Robot Data Exchange:**  
  Robots publish local maps and pose estimates to ROS2 topics. Time synchronization across topics minimizes latency effects.
  
- **Global Map Construction:**  
  Using Kalman filters and transformation matrices, local maps are fused into a coherent global representation:
  \[
  \hat{m}_{\text{global}} = \sum_{k} T_k \cdot m_k
  \]
  Global pose graphs are optimized to ensure consistency.

#### 2.4 Motion Control and Navigation

- **Trajectory Planning:**  
  Hybrid-A* generates feasible paths in real time. Cubic splines smooth the path:
  \[
  S(t) = \sum_{i=0}^{n} a_i t^i
  \]
  where coefficients \( a_i \) are determined to ensure continuity and smoothness.

- **Reactive Control:**  
  PID controllers adjust the robot's speed and orientation:
  \[
  u(t) = K_p e(t) + K_i \int_0^t e(\tau) \, d\tau + K_d \frac{de(t)}{dt}
  \]
  Combined with potential fields, the control input adapts to obstacles in real time.

#### 2.5 Adaptive Behavior and Map Refinement

- **Stochastic Re-Planning:**  
  When encountering obstacles, robots execute random rotational maneuvers integrated with deterministic control to escape local minima.
  
- **Iterative Map Update:**  
  The global map is continuously refined by re-optimizing the pose graph as new sensor data arrives.

### 3. Future Work and Reinforcement Learning Integration

#### 3.1 Reinforcement Learning (RL) for Adaptive Navigation and Task Allocation

- **Action Space Expansion:**  
  Extend the control framework to include adaptive speed, dynamic goal switching, and environmental interaction. The action space \( A \) will include both continuous and discrete actions.

- **Reward Function Design:**  
  Define rewards to balance exploration and exploitation:
  \[
  R(s,a) = \alpha \cdot \text{Coverage} - \beta \cdot \text{Collision Penalty} - \gamma \cdot \text{Energy Consumption}
  \]
  with tunable parameters \( \alpha, \beta, \gamma \).

- **RL Algorithm:**  
  Implement Multi-Agent Proximal Policy Optimization (MAPPO) to allow decentralized learning. The policy \( \pi_\theta(a|s) \) is updated using:
  \[
  \theta \leftarrow \theta + \eta \nabla_\theta \mathbb{E} \left[ \min\left(r(\theta) A, \text{clip}(r(\theta), 1-\epsilon, 1+\epsilon) A\right) \right]
  \]
  where \( r(\theta) \) is the probability ratio and \( A \) is the advantage function.

- **Simulation-Based Training and Policy Transfer:**  
  Train RL models in simulation (Gazebo, RViz) and employ domain randomization for robust transfer to real-world robots. The RL module will act as a high-level decision-maker, while low-level control remains with traditional PID methods.

#### 3.2 Expanded Swarm and Perception Capabilities

- **Advanced Coordination:**  
  Enhance inter-robot consensus using decentralized algorithms to improve task allocation and formation control.

- **Enhanced Sensor Fusion:**  
  Integrate deep learning models for real-time object detection and scene understanding to further refine SLAM and mapping accuracy.

- **Human-Robot Collaboration:**  
  Develop intuitive interfaces and communication protocols to facilitate seamless integration between autonomous systems and human operators for cooperative task execution.

---

This README provides a detailed, mathematically rigorous explanation of the multi-robot collaborative mapping framework. It outlines the key concepts, technical methods, step-by-step working of the system, and future directions, ensuring the project is scalable, robust, and at the cutting edge of robotics research.

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