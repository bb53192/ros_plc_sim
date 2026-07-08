#!/usr/bin/env python3
"""Gazebo Harmonic sim for FANUC CRX-10iA with a Robotiq 2F-85 mounted on the flange.

No MoveIt stack here yet -- this just spawns the arm+gripper in the (empty for now)
gripper_cell world and brings up ros2_control. MoveIt/SRDF for the combined arm+gripper
comes later once the cell has fixtures to plan around.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
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


def _configure(context):
    pkg_gazebo = get_package_share_directory("ros_plc_sim")
    pkg_crx_desc = get_package_share_directory("fanuc_crx_description")
    pkg_robotiq_desc = get_package_share_directory("robotiq_description")
    gz_resource_path = ":".join(
        [os.path.dirname(pkg_crx_desc), os.path.dirname(pkg_robotiq_desc)]
    )
    urdf_file = os.path.join(pkg_gazebo, "urdf", "crx10ia_robotiq_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers_gripper.yaml")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            TextSubstitution(text=" "),
            TextSubstitution(text=urdf_file),
            TextSubstitution(text=" simulation_controllers:="),
            TextSubstitution(text=controllers_yaml),
            TextSubstitution(text=" use_robotiq_gripper:="),
            LaunchConfiguration("use_robotiq_gripper"),
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "crx10ia",
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.05",
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
                "config_file": os.path.join(pkg_gazebo, "config", "gz_bridge.yaml"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    delayed_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    # TrackController has no default speed, so kick the belt once the world is up.
    # Negative == drives the part along -X toward the robot/end-stop. (0 stops it.)
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
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo GUI",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ros_plc_sim"), "worlds", "gripper_cell.sdf"]
                ),
                description="Gazebo world file",
            ),
            DeclareLaunchArgument(
                "use_robotiq_gripper",
                default_value="true",
                description="Attach the Robotiq 2F-85 gripper to the CRX-10iA flange",
            ),
            OpaqueFunction(function=_configure),
        ]
    )
