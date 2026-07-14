#!/usr/bin/env python3
"""Full PLC-facing cell: Gazebo cell + MoveIt + cell_io orchestrator.

= crx_gripper_moveit.launch.py (cell + move_group, belt NOT auto-started) + cell_io, which
exposes the plain std_msgs/Bool PLC contract (/plc/belt_run, /plc/robot_start ->
/sensors/part_present, /sensors/robot_busy, /sensors/cycle_done). Point the plc_bridge
(OPC UA + Modbus) at these topics.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ros_plc_sim")

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "crx_gripper_moveit.launch.py")),
        launch_arguments={
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "autostart_belt": "false",  # cell_io / the PLC owns the belt
        }.items(),
    )

    # AUTO/MANUAL arbiter: picks PLC (/plc/*) vs HMI (/manual/*) command source
    # and republishes onto /cmd/*. Defaults to MANUAL (safe) — flip to AUTO via
    # the /cell/set_mode service. plc_bridge stays unchanged (still on /plc/*).
    cell_mode = Node(
        package="ros_plc_sim",
        executable="cell_mode.py",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"start_mode": LaunchConfiguration("start_mode")},
            {"manual_source": LaunchConfiguration("manual_source")},
        ],
    )

    # cell_io needs /compute_ik (move_group); start it after the stack settles.
    # Its command inputs are remapped onto the arbiter's /cmd/* output.
    cell_io = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="ros_plc_sim",
                executable="cell_io.py",
                output="screen",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    ("/plc/belt_run", "/cmd/belt_run"),
                    ("/plc/robot_start", "/cmd/robot_start"),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("start_mode", default_value="manual"),
            DeclareLaunchArgument("manual_source", default_value="ros"),
            cell,
            cell_mode,
            cell_io,
        ]
    )
