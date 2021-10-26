#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='platform_control',
            executable='platform_control.py',
            name='platform_control',
            parameters=[],
            # output='screen'

        ),

        Node(
            package='rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': '/dev/ttyUSB1',
                         'serial_baudrate': 256000,  # A3
                         'frame_id': 'lidar_1_raw',
                         'topic_name': '/scan_1_raw',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Sensitivity',
                         }],
            # output='screen'
        ),

        Node(
            package='rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 256000,  # A3
                         'frame_id': 'lidar_2_raw',
                         'topic_name': '/scan_2_raw',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Sensitivity',
                         }],
            # output='screen'
        ),

        Node(
            package='laserscan_merger',
            executable='laserscan_merger',
            name='laserscan_merger',
            parameters=[{
                "destination_frame": "lidar_1",
                "cloud_destination_topic": "/cloud",
                "scan_destination_topic": "/scan_1",
                "laserscan_topics": "/scan_1_raw",
                "mode_name": "/platform_mode",
                "angle_min": -1.5707,
                "angle_max":  3.1415,
                "angle_increment": 0.0058,
                "scan_time": 0.033,
                "range_min": 0.0,
                "range_max": 25.0,
                "verbose": False,
                "timeout": 20.0
            }]),

        Node(
            package='laserscan_merger',
            executable='laserscan_merger',
            name='laserscan_merger',
            parameters=[{
                "destination_frame": "lidar_2",
                "cloud_destination_topic": "/cloud",
                "scan_destination_topic": "/scan_2",
                "laserscan_topics": "/scan_2_raw",
                "mode_name": "/platform_mode",
                "angle_min": -1.5707,
                "angle_max":  3.1415,
                "angle_increment": 0.0058,
                "scan_time": 0.033,
                "range_min": 0.0,
                "range_max": 25.0,
                "verbose": False,
                "timeout": 20.0
            }]),


        Node(
            package='laserscan_merger',
            executable='laserscan_merger',
            name='laserscan_merger',
            parameters=[{
                "destination_frame": "robot_base",
                "cloud_destination_topic": "/cloud",
                "scan_destination_topic": "/scan",
                "laserscan_topics": "/scan_1 /scan_2",
                "mode_name": "/platform_mode",
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.0058,
                "scan_time": 0.033,
                "range_min": 0.2,
                "range_max": 25.0,
                "verbose": False,
                "timeout": 20.0
            }]),

    ])
