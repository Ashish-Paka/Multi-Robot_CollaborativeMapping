
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
