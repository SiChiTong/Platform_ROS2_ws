# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch

package_name = 'detection_2d'

def generate_launch_description():
    package_prefix = get_package_share_directory(package_name)
    pkg_executable = "detect_yolo.py"

    pcd_pkg_name = 'pcd_manager'
    pcd_executable = "pcd_publisher.py"

    pcd_cmd = Node(
        package=pcd_pkg_name,
        executable=pcd_executable,
        name='pcd_publisher',
        output='screen',
        parameters=[{'show_FPS': False}])

    pkg_cmd = Node(
        package=package_name,
        executable=pkg_executable,
        name='detect_yolo',
        output='screen',
        parameters=[{'show_FPS': False}])

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(pcd_cmd)
    ld.add_action(pkg_cmd)

    return ld
