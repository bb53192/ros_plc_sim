#!/usr/bin/env python3
"""Bring up the full CRX-10iA + Robotiq gripper cell AND MoveIt together.

= crx_gripper_gz_sim.launch.py (Gazebo cell + ros2_control) + crx_gripper_moveit_stack.launch.py
(move_group + RViz). This is the M1 entry point: plan/execute the arm in RViz against the cell.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("ros_plc_sim")

    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    autostart_belt = LaunchConfiguration("autostart_belt")

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "crx_gripper_gz_sim.launch.py")),
        launch_arguments={
            "gazebo_gui": gazebo_gui,
            "autostart_belt": autostart_belt,
        }.items(),
    )

    # move_group needs the controllers up; give the cell a head start.
    moveit = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "crx_gripper_moveit_stack.launch.py")
                ),
                launch_arguments={
                    "launch_rviz": launch_rviz,
                    "use_sim_time": "true",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("autostart_belt", default_value="true"),
            cell,
            moveit,
        ]
    )
