#!/usr/bin/env python3
"""Launch file for autonomous warehouse robots with LiDAR obstacle avoidance.

This launch file starts:
1. Gazebo Classic with the warehouse world
2. Robot State Publishers for robot1, robot2, and robot3 (publishes /robotN/robot_description and TF)
3. Spawns robot1, robot2, and robot3 in Gazebo
4. Starts obstacle avoidance nodes for robot1, robot2, and robot3
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

    # Process xacro three times — once per robot — with distinct namespace/tf_prefix mappings
    robot1_description = xacro.process_file(
        xacro_file,
        mappings={'robot_namespace': 'robot1', 'tf_prefix': 'robot1_'}
    ).toxml()

    robot2_description = xacro.process_file(
        xacro_file,
        mappings={'robot_namespace': 'robot2', 'tf_prefix': 'robot2_'}
    ).toxml()

    robot3_description = xacro.process_file(
        xacro_file,
        mappings={'robot_namespace': 'robot3', 'tf_prefix': 'robot3_'}
    ).toxml()

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
    # 2. Robot State Publishers — publish /robotN/robot_description and TF frames
    # -------------------------------------------------------------------------

    # robot1 State Publisher
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[
            {
                'robot_description': robot1_description,
                'use_sim_time': True,
                'frame_prefix': 'robot1_',
            }
        ]
    )

    # robot2 State Publisher
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[
            {
                'robot_description': robot2_description,
                'use_sim_time': True,
                'frame_prefix': 'robot2_',
            }
        ]
    )

    # robot3 State Publisher
    robot3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot3',
        output='screen',
        parameters=[
            {
                'robot_description': robot3_description,
                'use_sim_time': True,
                'frame_prefix': 'robot3_',
            }
        ]
    )

    # -------------------------------------------------------------------------
    # 3. Spawn robots in Gazebo (delayed to let Gazebo initialise)
    # -------------------------------------------------------------------------

    # Spawn robot1 (delayed 5 s)
    spawn_robot1 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot1',
                output='screen',
                arguments=[
                    '-topic', '/robot1/robot_description',
                    '-entity', 'robot1',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                    '-Y', '0.0',
                ]
            )
        ]
    )

    # Spawn robot2 (delayed 6 s, after robot1)
    spawn_robot2 = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot2',
                output='screen',
                arguments=[
                    '-topic', '/robot2/robot_description',
                    '-entity', 'robot2',
                    '-x', '2.0',
                    '-y', '0.0',
                    '-z', '0.1',
                    '-Y', '0.0',
                ]
            )
        ]
    )

    # Spawn robot3 (delayed 7 s, after robot2)
    spawn_robot3 = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot3',
                output='screen',
                arguments=[
                    '-topic', '/robot3/robot_description',
                    '-entity', 'robot3',
                    '-x', '4.0',
                    '-y', '0.0',
                    '-z', '0.1',
                    '-Y', '0.0',
                ]
            )
        ]
    )

    # -------------------------------------------------------------------------
    # 4. Obstacle avoidance nodes (all delayed 10 s to let robots spawn first)
    # -------------------------------------------------------------------------

    # robot1 obstacle avoidance
    obstacle_avoidance_robot1 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='warehouse_robot',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                namespace='robot1',
                output='screen',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('scan', '/robot1/scan'),
                    ('cmd_vel', '/robot1/cmd_vel'),
                ]
            )
        ]
    )

    # robot2 obstacle avoidance
    obstacle_avoidance_robot2 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='warehouse_robot',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                namespace='robot2',
                output='screen',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('scan', '/robot2/scan'),
                    ('cmd_vel', '/robot2/cmd_vel'),
                ]
            )
        ]
    )

    # robot3 obstacle avoidance
    obstacle_avoidance_robot3 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='warehouse_robot',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                namespace='robot3',
                output='screen',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('scan', '/robot3/scan'),
                    ('cmd_vel', '/robot3/cmd_vel'),
                ]
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        robot1_state_publisher,
        robot2_state_publisher,
        robot3_state_publisher,
        spawn_robot1,
        spawn_robot2,
        spawn_robot3,
        obstacle_avoidance_robot1,
        obstacle_avoidance_robot2,
        obstacle_avoidance_robot3,
    ])
