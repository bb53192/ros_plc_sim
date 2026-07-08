#!/usr/bin/env python3
"""MoveIt move_group + RViz for the CRX-10iA + Robotiq gripper cell.

Mirrors fanuc_gazebo/launch/fanuc_moveit_stack.launch.py, but builds the config against the
arm+gripper URDF (crx10ia_robotiq_gz.urdf.xacro) and this package's SRDF, so move_group knows
about the gripper. Expects the Gazebo cell + controllers to already be running
(crx_gripper_gz_sim.launch.py); use crx_gripper_moveit.launch.py to bring up both together.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    pkg = get_package_share_directory("ros_plc_sim")
    urdf_file = os.path.join(pkg, "urdf", "crx10ia_robotiq_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg, "config", "ros2_controllers_gripper.yaml")
    moveit_controllers = os.path.join(pkg, "config", "moveit", "moveit_controllers.yaml")

    moveit_config = (
        MoveItConfigsBuilder("crx10ia", package_name="ros_plc_sim")
        .robot_description(
            file_path=urdf_file,
            mappings={
                "simulation_controllers": controllers_yaml,
                "use_robotiq_gripper": "true",
            },
        )
        .robot_description_semantic(file_path="srdf/crx10ia_robotiq.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .trajectory_execution(file_path=moveit_controllers)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))
    ld.add_action(DeclareLaunchArgument("launch_rviz", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(pkg, "config", "moveit", "moveit.rviz"),
        )
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                move_group_configuration,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    )

    return ld
