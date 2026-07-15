#!/usr/bin/env python3
"""Full two-arm color-sorting demo: Gazebo cell + MoveIt IK + cameras + classifiers + orchestrator.

= dual_arm_sort_moveit.launch.py (cell + move_group, belt NOT auto-started; the orchestrator owns
it) + one color_classifier per station + the sort_cell orchestrator. Mirrors the single-arm
crx_gripper_plc.launch.py structure (staged TimerActions so each layer waits for the one below).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _classifier(station):
    return Node(
        package="ros_plc_sim",
        executable="color_classifier.py",
        name=f"color_classifier_{station}",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"image_topic": f"/station_{station}/image"},
            {"output_topic": f"/station_{station}/part_color"},
        ],
    )


def generate_launch_description():
    pkg = get_package_share_directory("ros_plc_sim")

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "dual_arm_sort_moveit.launch.py")),
        launch_arguments={
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "autostart_belt": "false",  # sort_cell owns the belt
        }.items(),
    )

    classifiers = TimerAction(period=10.0, actions=[_classifier("a"), _classifier("b")])

    # sort_cell needs /compute_ik (move_group) + the controllers; start it after the stack settles.
    sort_cell = TimerAction(
        period=16.0,
        actions=[
            Node(
                package="ros_plc_sim",
                executable="sort_cell.py",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"force_color": LaunchConfiguration("force_color")},
                    {"on_the_fly": LaunchConfiguration("on_the_fly")},
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "force_color", default_value="",
                description="Debug: force every part's color (blue/green/red/yellow); empty = random",
            ),
            DeclareLaunchArgument(
                "on_the_fly", default_value="false",
                description="Experimental: green arm grabs the part in motion (belt never stops)",
            ),
            cell,
            classifiers,
            sort_cell,
        ]
    )
