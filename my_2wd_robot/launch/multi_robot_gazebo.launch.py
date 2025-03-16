import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define the robot spawn parameters for two robots
    robots = [
        {'namespace': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.0'},
        {'namespace': 'robot2', 'x': '2.0', 'y': '0.0', 'z': '0.0'},
    ]

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    ld = LaunchDescription()
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

        # Add nodes to the launch description
        ld.add_action(rsp_node)
        ld.add_action(spawn_entity)

    return ld
    