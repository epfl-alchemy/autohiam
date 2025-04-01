"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handeye_calibration',
            executable='robot',
            name='robot_state_estimation'
        ),
        Node(
            package='handeye_calibration',
            executable='marker',
            name='aruco_estimation'
        ),
    ])