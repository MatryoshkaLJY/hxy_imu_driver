# SPDX-License-Identifier: Apache-2.0
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = get_package_share_directory("hxy_imu_driver") + "/config/imu.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_config,
            description="Path to the IMU driver ROS parameter YAML file",
        ),
        Node(
            package="hxy_imu_driver",
            executable="imu_node",
            name="hxy_imu",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
        ),
    ])
