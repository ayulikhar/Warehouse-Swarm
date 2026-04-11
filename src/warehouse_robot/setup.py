from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

        # URDF
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),

        # Worlds
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='Autonomous warehouse robot with LiDAR obstacle avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = warehouse_robot.obstacle_avoidance_node:main',
            'swarm_controller_node = warehouse_robot.swarm_controller_node:main',
        ],
    },
)
