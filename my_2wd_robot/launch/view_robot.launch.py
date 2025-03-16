import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory


def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('my_2wd_robot'),
        'urdf',
        'my_robot.urdf.xacro'
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('my_2wd_robot'), 'rviz', 'description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        
        DeclareLaunchArgument(
            name='publish_joints', 
            default_value='false',
            description='Launch joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='publish_frequency', 
            default_value='50.0',
            description='Publish Frequency'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')]),
                    'publish_frequency': LaunchConfiguration('publish_frequency')
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
