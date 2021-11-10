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

package_name = 'platform_nav2'

def generate_launch_description():
    package_prefix = get_package_share_directory(package_name)

    gui_prefix = get_package_share_directory('platform_gui')
    gui_executable = "platform_gui"

    commander_launch_file_dir = os.path.join(package_prefix)
    commander_executable = "nav2_commander.py"

    robot_name = LaunchConfiguration('robot_base', default="robot_base")
    visualization_map = LaunchConfiguration('visualization_map', default=False)
    visualization_bbox = LaunchConfiguration('visualization_bbox', default=True)

    gui_cmd = Node(
        package='platform_gui',
        executable=gui_executable,
        name='platform_gui',
        output='screen',
        parameters=[])

    commander_cmd = Node(
        package='platform_nav2',
        executable=commander_executable,
        name='nav2_commander',
        output='screen',
        parameters=[{"robot_base": robot_name,
                     "visualization_map" : visualization_map,
                     "visualization_bbox" : visualization_bbox,

    }])

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gui_cmd)
    # ld.add_action(commander_cmd)

    return ld