#!/usr/bin/env python3
"""Launch file for autonomous warehouse robot with LiDAR obstacle avoidance.

This launch file starts:
1. Gazebo Classic with the warehouse world
2. Robot State Publisher (publishes /robot_description and TF)
3. Spawns the robot in Gazebo
4. Starts the obstacle avoidance node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_robot')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'warehouse_robot.urdf.xacro')

    # Process xacro to get robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # -------------------------------------------------------------------------
    # 1. Launch Gazebo Classic with the warehouse world
    # -------------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'pause': 'false',
        }.items()
    )

    # -------------------------------------------------------------------------
    # 2. Robot State Publisher — publishes /robot_description and TF frames
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )

    # -------------------------------------------------------------------------
    # 3. Spawn robot in Gazebo (delayed 5 s to let Gazebo initialise)
    # -------------------------------------------------------------------------
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',
                    '-entity', 'warehouse_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                    '-Y', '0.0',
                ]
            )
        ]
    )

    # -------------------------------------------------------------------------
    # 4. Obstacle avoidance node (delayed 8 s to let robot spawn first)
    # -------------------------------------------------------------------------
    obstacle_avoidance = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='warehouse_robot',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        obstacle_avoidance,
    ])
