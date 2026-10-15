#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_config.launch_config import load_launch_config


def _compose(context):
    share = get_package_share_directory("robot_state_estimation")
    requested = LaunchConfiguration("base_fusion_mode").perform(context).strip().lower()
    effective_params = LaunchConfiguration("effective_params_file").perform(context).strip()
    effective = load_launch_config(effective_params)[0] if effective_params else None
    compiled_mode = effective["runtime"]["fusion_mode"] if effective else "wheel"
    aliases = {"none": "wheel", "ekf": "wheel_imu"}
    fusion_mode = aliases.get(requested, requested) if requested else compiled_mode
    if effective and fusion_mode != compiled_mode:
        raise RuntimeError("base_fusion_mode differs from compiled config")
    if fusion_mode not in ("wheel", "wheel_imu"):
        raise RuntimeError("base_fusion_mode must be wheel or wheel_imu")
    node_params = [effective_params] if effective_params else []
    nodes = [
        Node(
            package="robot_state_estimation",
            executable="wheel_odometry_node.py",
            name="wheel_odometry",
            output="screen",
            parameters=node_params,
        ),
        Node(
            package="robot_state_estimation",
            executable="imu_adapter_node.py",
            name="imu_adapter",
            output="screen",
            parameters=node_params,
        ),
    ]
    formal_input = effective["platform"]["topics"]["wheel_odom"] if effective else "wheel/odom"
    if fusion_mode == "wheel_imu":
        ekf_file = LaunchConfiguration("ekf_wheel_imu_params_file").perform(context)
        if effective:
            ekf_file = os.path.join(os.path.dirname(effective_params), "ekf-wheel-imu.yaml")
        if not os.path.isfile(ekf_file):
            raise RuntimeError(f"EKF params file does not exist: {ekf_file}")
        nodes.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="base_ekf",
                output="screen",
                parameters=[ekf_file],
                remappings=[("odometry/filtered", "odometry/filtered_internal")],
            )
        )
        formal_input = "odometry/filtered_internal"
    nodes.append(
        Node(
            package="robot_state_estimation",
            executable="formal_odometry_node.py",
            name="formal_odometry",
            output="screen",
            parameters=[
                *node_params,
                {
                    "input_topic": formal_input,
                    "output_topic": (
                        effective["platform"]["topics"]["odom"]
                        if effective
                        else LaunchConfiguration("odom_topic")
                    ),
                },
            ],
        )
    )
    return nodes


def generate_launch_description():
    share = get_package_share_directory("robot_state_estimation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("base_fusion_mode", default_value=""),
            DeclareLaunchArgument("effective_params_file", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument(
                "ekf_wheel_params_file",
                default_value=os.path.join(share, "config", "ekf_wheel.yaml"),
            ),
            DeclareLaunchArgument(
                "ekf_wheel_imu_params_file",
                default_value=os.path.join(share, "config", "ekf_base.yaml"),
            ),
            OpaqueFunction(function=_compose),
        ]
    )
