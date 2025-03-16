import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    launch_gazebo= os.path.join(
                get_package_share_directory('my_2wd_robot'),  
                'launch',
                'robot_gazebo.launch.py'  
            )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("my_2wd_robot"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    # Add a delay for the second launch file (e.g., 5 seconds)
    delayed_slam_toolbox_node = TimerAction(
        period=15.0,  # Delay in seconds
        actions=[start_async_slam_toolbox_node]
    )
    

     #Include another launch file
    launch_gazebo_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_gazebo)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(delayed_slam_toolbox_node)
    ld.add_action(launch_gazebo_rviz)

    return ld
