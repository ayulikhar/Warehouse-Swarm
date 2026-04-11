#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_robot')

    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'warehouse_robot.urdf.xacro')

    # TWO robot descriptions (same model)
    robot1_desc = xacro.process_file(xacro_file)
    robot2_desc = xacro.process_file(xacro_file)

    # ------------------ GAZEBO ------------------
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

    # ------------------ STATE PUBLISHERS ------------------
    robot1_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': robot1_desc.toxml(),
            'use_sim_time': True
        }],
        output='screen'
    )

    robot2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': robot2_desc.toxml(),
            'use_sim_time': True
        }],
        output='screen'
    )

    # ------------------ SPAWN ROBOTS ------------------
    spawn_robot1 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', '/robot1/robot_description',
                    '-entity', 'robot1',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                ],
                output='screen'
            )
        ]
    )

    spawn_robot2 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', '/robot2/robot_description',
                    '-entity', 'robot2',
                    '-x', '1.5',
                    '-y', '0.0',
                    '-z', '0.1',
                ],
                output='screen'
            )
        ]
    )

    # ------------------ NODES ------------------
#    node1 = TimerAction(
#        period=8.0,
#        actions=[
#            Node(
#                package='warehouse_robot',
#                executable='obstacle_avoidance_node',
#                namespace='robot1',
#                output='screen'
#            )
#        ]
#    )

#    node2 = TimerAction(
#        period=8.0,
#        actions=[
#            Node(
#                package='warehouse_robot',
#                executable='obstacle_avoidance_node',
#                namespace='robot2',
#                output='screen'
#            )
#        ]
#    )

    return LaunchDescription([
        gazebo_launch,
        robot1_rsp,
        robot2_rsp,
        spawn_robot1,
        spawn_robot2,
#        node1,
#        node2,
    ])
