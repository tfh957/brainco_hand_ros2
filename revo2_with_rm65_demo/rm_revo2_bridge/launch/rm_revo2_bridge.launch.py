#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_bridge_env():
    env = {"DISPLAY": os.environ.get("DISPLAY", "")}
    rm_iface_prefix = "/home/fishros/rmrobot/rm_ws/install/rm_ros_interfaces"
    py_path = f"{rm_iface_prefix}/local/lib/python3.10/dist-packages"
    lib_path = f"{rm_iface_prefix}/lib"

    existing_py = os.environ.get("PYTHONPATH", "")
    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    existing_ament = os.environ.get("AMENT_PREFIX_PATH", "")
    existing_cmake = os.environ.get("CMAKE_PREFIX_PATH", "")

    env["PYTHONPATH"] = py_path + (os.pathsep + existing_py if existing_py else "")
    env["LD_LIBRARY_PATH"] = lib_path + (os.pathsep + existing_ld if existing_ld else "")
    env["AMENT_PREFIX_PATH"] = rm_iface_prefix + (os.pathsep + existing_ament if existing_ament else "")
    env["CMAKE_PREFIX_PATH"] = rm_iface_prefix + (os.pathsep + existing_cmake if existing_cmake else "")
    return env


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rm_robot_ip", default_value="192.168.1.18"),
            DeclareLaunchArgument("trajectory_topic", default_value="/revo2_hand_controller/joint_trajectory"),
            DeclareLaunchArgument("hand_backend", default_value="rm_driver"),
            DeclareLaunchArgument("device_id", default_value="127"),
            DeclareLaunchArgument("max_joint_rad", default_value="1.7453292519943295"),
            DeclareLaunchArgument("arm_follow", default_value="false"),
            DeclareLaunchArgument("timeout_100ms", default_value="20"),
            DeclareLaunchArgument("hand_write_retry_count", default_value="1"),
            DeclareLaunchArgument("hand_reinit_on_write_fail", default_value="true"),
            DeclareLaunchArgument("feedback_enabled", default_value="false"),
            DeclareLaunchArgument("publish_command_to_joint_states", default_value="true"),
            DeclareLaunchArgument("publish_hand_state_to_joint_states_rate_hz", default_value="15.0"),
            DeclareLaunchArgument(
                "robotic_arm_package_path",
                default_value="/home/fishros/rmrobot/2ndHandDemo/2ndHandDemo",
            ),
            Node(
                package="rm_revo2_bridge",
                executable="rm_revo2_bridge_node",
                output="screen",
                additional_env=_build_bridge_env(),
                parameters=[
                    {
                        "rm_robot_ip": LaunchConfiguration("rm_robot_ip"),
                        "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                        "hand_backend": LaunchConfiguration("hand_backend"),
                        "device_id": LaunchConfiguration("device_id"),
                        "max_joint_rad": LaunchConfiguration("max_joint_rad"),
                        "arm_follow": LaunchConfiguration("arm_follow"),
                        "timeout_100ms": LaunchConfiguration("timeout_100ms"),
                        "hand_write_retry_count": LaunchConfiguration("hand_write_retry_count"),
                        "hand_reinit_on_write_fail": LaunchConfiguration("hand_reinit_on_write_fail"),
                        "feedback_enabled": LaunchConfiguration("feedback_enabled"),
                        "publish_command_to_joint_states": LaunchConfiguration("publish_command_to_joint_states"),
                        "publish_hand_state_to_joint_states_rate_hz": LaunchConfiguration(
                            "publish_hand_state_to_joint_states_rate_hz"
                        ),
                        "robotic_arm_package_path": LaunchConfiguration("robotic_arm_package_path"),
                    }
                ],
            ),
        ]
    )
