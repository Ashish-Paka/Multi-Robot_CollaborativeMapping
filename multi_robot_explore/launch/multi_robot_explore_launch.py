# from launch import LaunchDescription
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Get the path to the params folder in the package
#     package_name = 'multi_robot_explore'
#     params_dir = os.path.join(
#         get_package_share_directory(package_name), 'params'
#     )

#     return LaunchDescription([
#         # Launch BallisticMover for robot1 with its params file
#         Node(
#             package=package_name,
#             executable='ballistic_mover',
#             name='robot1',
#             output='screen',
#             parameters=[
#                 {os.path.join(params_dir, 'robot_params.yaml')},
#                 {'robot_name': 'robot2'}
#                 ]
#         ),
#         # Launch BallisticMover for robot2 with its params file
#         # Node(
#         #     package=package_name,
#         #     executable='ballistic_mover',
#         #     name='robot2',
#         #     parameters=[os.path.join(params_dir, 'robot2_params.yaml')]
#         # )
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'multi_robot_explore'
    params_dir = os.path.join(
        get_package_share_directory(package_name), 'params'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='ballistic_mover',
            name='robot2',
            output='screen',
            parameters=[
                os.path.join(params_dir, 'robot_params.yaml'),  # parameter file as a string
                {'robot_name': 'robot2'}                        # additional parameter override as a dictionary
            ]
        )
    ])
