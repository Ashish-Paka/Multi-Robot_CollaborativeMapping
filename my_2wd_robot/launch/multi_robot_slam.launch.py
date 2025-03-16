import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ld = LaunchDescription()
    
    # Define the robot spawn parameters for two robots
    robots = [
        {'namespace': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'rviz_config': 'description1.rviz'},
        {'namespace': 'robot2', 'x': '2.0', 'y': '0.0', 'z': '0.0', 'rviz_config': 'description2.rviz'},
    ]

    world_file = PathJoinSubstitution(
        [FindPackageShare('my_2wd_robot'), 'world', 'testing.world']  # Replace with your world file
    )

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', '-world', world_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(declare_sim_time_arg)

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
        rviz_config_path = robot['rviz_config']

        rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('my_2wd_robot'), 'rviz', rviz_config_path]
        )

        # Process the XACRO file
        doc = xacro.process_file(urdf_file)
        robot_desc = doc.toxml()

        # Write the URDF to a temporary file
        urdf_temp_file = os.path.join('/tmp', f'{namespace}.urdf')
        with open(urdf_temp_file, 'w') as f:
            f.write(robot_desc)

        # Set robot_description parameter
        robot_description = {'robot_description': robot_desc}

        # Start robot_state_publisher node under the robot's namespace
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )

        # Spawn the robot in Gazebo (remove '-urdf' flag)
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_' + namespace,
            output='screen',
            arguments=[
                '-file', urdf_temp_file,
                '-entity', namespace,
                '-robot_namespace', namespace,
                '-x', x,
                '-y', y,
                '-z', z,
            ]
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}]
        )


        # slam_params_file = PathJoinSubstitution(
        #     [FindPackageShare('my_2wd_robot'), 'config', f'mapper_params_online_async_{namespace}.yaml']
        # )
        # declare_slam_params_file_cmd = DeclareLaunchArgument(
        # f'slam_params_file_{namespace}',
        # default_value=slam_params_file,
        # description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

        # start_async_slam_toolbox_node = Node(
        #     parameters=[
        #     {'use_sim_time': use_sim_time, 'slam_params_file': slam_params_file}
        #     ],
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name=f'slam_toolbox_{namespace}',
        #     output='screen'
        # )

        # delayed_slam_toolbox_node = TimerAction(
        # period=20.0,  # Delay in seconds
        # actions=[start_async_slam_toolbox_node]
        # )

        # Add nodes to the launch description
        ld.add_action(rsp_node)
        ld.add_action(spawn_entity)
        ld.add_action(rviz)
        # ld.add_action(delayed_slam_toolbox_node)

    return ld
    