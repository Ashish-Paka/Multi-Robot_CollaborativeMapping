import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare launch arguments
    declare_lidar_enabled_cmd = DeclareLaunchArgument(
        'lidar_enabled',
        default_value='false',
        description='Enable the lidar sensor')
    
    declare_camera_enabled_cmd = DeclareLaunchArgument(
        'camera_enabled',
        default_value='false',
        description='Enable the camera sensor')
    
    # Define robot spawn parameters for two robots
    robots = [
        {'namespace': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'sensors': ['lidar']},
        {'namespace': 'robot2', 'x': '2.0', 'y': '0.0', 'z': '0.0', 'sensors': ['camera']},
    ]

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_lidar_enabled_cmd)
    ld.add_action(declare_camera_enabled_cmd)
    ld.add_action(gazebo)

    # Get the URDF/XACRO file path
    urdf_file = os.path.join(
        get_package_share_directory('my_2wd_robot'),
        'urdf',
        'my_robot.urdf.xacro'
    )

    for robot in robots:
        namespace = robot['namespace']
        x = robot['x']
        y = robot['y']
        z = robot['z']
        sensors = robot.get('sensors', [])

        # Set the sensor-specific parameters
        lidar_enabled = 'true' if 'lidar' in sensors else 'false'
        camera_enabled = 'true' if 'camera' in sensors else 'false'

        # Convert the Xacro file to URDF with sensor parameters
        robot_desc_cmd = Command(['xacro', urdf_file, 
                                  ' lidar_enabled:=', lidar_enabled,
                                  ' camera_enabled:=', camera_enabled])

        # Set robot_description parameter
        robot_description = {'robot_description': robot_desc_cmd}

        # Start robot_state_publisher node under the robot's namespace
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )

        # Spawn the robot in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_' + namespace,
            output='screen',
            arguments=[
                '-topic', f'/{namespace}/robot_description',
                '-entity', namespace,
                '-robot_namespace', namespace,
                '-x', x,
                '-y', y,
                '-z', z,
            ]
        )

        # Add nodes to the launch description
        ld.add_action(rsp_node)
        ld.add_action(spawn_entity)

    return ld
