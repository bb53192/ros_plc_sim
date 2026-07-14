#!/usr/bin/env python3
"""Gazebo Harmonic sim for the two-arm color-sorting cell.

Spawns the dual-arm description (dual_crx_gz.urdf.xacro: blue_ + green_ CRX-10iA + Robotiq) into
the sorting_cell world and brings up ros2_control (one controller_manager, four controllers +
joint_state_broadcaster). Mirrors crx_gripper_gz_sim.launch.py but for two arms. No MoveIt here;
use dual_arm_sort_moveit.launch.py to add /compute_ik, or dual_arm_sort.launch.py for the full
sorting demo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _spawner(name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )


def _configure(context):
    pkg_gazebo = get_package_share_directory("ros_plc_sim")
    pkg_crx_desc = get_package_share_directory("fanuc_crx_description")
    pkg_robotiq_desc = get_package_share_directory("robotiq_description")
    gz_resource_path = ":".join(
        [os.path.dirname(pkg_crx_desc), os.path.dirname(pkg_robotiq_desc)]
    )
    urdf_file = os.path.join(pkg_gazebo, "urdf", "dual_crx_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers_dual.yaml")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            TextSubstitution(text=" "),
            TextSubstitution(text=urdf_file),
            TextSubstitution(text=" simulation_controllers:="),
            TextSubstitution(text=controllers_yaml),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "dual_crx",
            "-allow_renaming", "true",
            "-x", "0", "-y", "0", "-z", "0.05",
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                LaunchConfiguration("gazebo_gui"),
                if_value=["-r -v 4 ", LaunchConfiguration("world_file")],
                else_value=["-s -r -v 4 ", LaunchConfiguration("world_file")],
            )
        }.items(),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_gazebo, "config", "gz_bridge_dual.yaml"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    delayed_spawners = TimerAction(
        period=5.0,
        actions=[
            _spawner("joint_state_broadcaster"),
            _spawner("blue_arm_controller"),
            _spawner("green_arm_controller"),
            _spawner("blue_gripper_controller"),
            _spawner("green_gripper_controller"),
        ],
    )

    # Kick the belt once the world is up (negative == toward the arms). Gated by autostart_belt:
    # false when the orchestrator (sort_cell) owns the belt.
    belt_start = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "gz", "topic",
                    "-t", "/model/belt_track/link/base_link/track_cmd_vel",
                    "-m", "gz.msgs.Double",
                    "-p", "data: -0.15",
                ],
                output="screen",
            )
        ],
        condition=IfCondition(LaunchConfiguration("autostart_belt")),
    )

    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=f"{gz_resource_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}",
        ),
        gz_sim,
        gz_spawn_entity,
        robot_state_publisher_node,
        gz_bridge,
        delayed_spawners,
        belt_start,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start Gazebo GUI"),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ros_plc_sim"), "worlds", "sorting_cell.sdf"]
                ),
                description="Gazebo world file",
            ),
            DeclareLaunchArgument(
                "autostart_belt",
                default_value="true",
                description="Publish a default belt speed at startup. Set false when the orchestrator drives the belt.",
            ),
            OpaqueFunction(function=_configure),
        ]
    )
