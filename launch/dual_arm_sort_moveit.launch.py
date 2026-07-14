#!/usr/bin/env python3
"""Two-arm sorting cell (Gazebo) + a single MoveIt move_group serving /compute_ik for both arms.

= dual_arm_sort_gz.launch.py (Gazebo cell + dual ros2_control) + a move_group built from the dual
URDF/SRDF. MoveIt here is used ONLY as an IK solver (groups blue_manipulator / green_manipulator);
the orchestrator drives the controllers directly, same as the single-arm cell. Mirrors
crx_gripper_moveit.launch.py + crx_gripper_moveit_stack.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("ros_plc_sim")
    urdf_file = os.path.join(pkg, "urdf", "dual_crx_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg, "config", "ros2_controllers_dual.yaml")
    moveit_controllers = os.path.join(pkg, "config", "moveit", "moveit_controllers_dual.yaml")

    moveit_config = (
        MoveItConfigsBuilder("dual_crx", package_name="ros_plc_sim")
        .robot_description(
            file_path=urdf_file,
            mappings={"simulation_controllers": controllers_yaml},
        )
        .robot_description_semantic(file_path="srdf/dual_crx_robotiq.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics_dual.yaml")
        .joint_limits(file_path="config/moveit/joint_limits_dual.yaml")
        .trajectory_execution(file_path=moveit_controllers)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "dual_arm_sort_gz.launch.py")),
        launch_arguments={
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "autostart_belt": LaunchConfiguration("autostart_belt"),
        }.items(),
    )

    move_group = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"publish_robot_description_semantic": True},
                    {"use_sim_time": True},
                ],
            )
        ],
    )

    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_moveit",
                output="log",
                arguments=["-d", os.path.join(pkg, "config", "moveit", "moveit.rviz")],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("autostart_belt", default_value="true"),
            cell,
            move_group,
            rviz,
        ]
    )
