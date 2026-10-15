#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("imu_frame", default_value="imu_link"),
            DeclareLaunchArgument("laser_frame", default_value="laser_frame"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("lidar_tf_x", default_value="0.07"),
            DeclareLaunchArgument("lidar_tf_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_z", default_value="0.13"),
            DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_tf_yaw", default_value="1.570796326795"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_base_footprint",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    LaunchConfiguration("base_frame"),
                    "base_footprint",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_imu_link",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    LaunchConfiguration("base_frame"),
                    LaunchConfiguration("imu_frame"),
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub_laser",
                condition=IfCondition(LaunchConfiguration("use_lidar")),
                arguments=[
                    LaunchConfiguration("lidar_tf_x"),
                    LaunchConfiguration("lidar_tf_y"),
                    LaunchConfiguration("lidar_tf_z"),
                    LaunchConfiguration("lidar_tf_yaw"),
                    LaunchConfiguration("lidar_tf_pitch"),
                    LaunchConfiguration("lidar_tf_roll"),
                    LaunchConfiguration("base_frame"),
                    LaunchConfiguration("laser_frame"),
                ],
                output="screen",
            ),
        ]
    )
