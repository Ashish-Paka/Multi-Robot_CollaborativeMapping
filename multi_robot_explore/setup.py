from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'multi_robot_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vedant',
    maintainer_email='vedantchoudhary16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'multi_robot_random_walk = multi_robot_explore.multi_robot_random_explore:main',
            'multi_robot_random_walk_robot1 = multi_robot_explore.multi_robot_random_explore_robot1:main',
            'multi_robot_random_walk_robot2 = multi_robot_explore.multi_robot_random_explore_robot2:main',
            # 'test_robot = multi_robot_explore.test:main',
            'ballistic_mover = multi_robot_explore.multi_robot_random_explore_test_2:main',
        ],
    },
)
