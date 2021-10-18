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

os.environ['TURTLEBOT3_MODEL'] = "waffle"
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    package_prefix = get_package_share_directory(package_name)

    platform_control_prefix = get_package_share_directory('platform_control')

    map_file = "map.yaml"
    map_dir = LaunchConfiguration('map_file',
                                  default=os.path.join(platform_control_prefix, 'map', map_file))

    param_file_name = 'nav2_platform.yaml'
    param_dir = LaunchConfiguration('params_file',
                                    default=os.path.join(package_prefix,'config',param_file_name))

    nav_to_pose_bt_xml_file = "nav_test.xml"
    nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml',
                                 default=os.path.join(package_prefix, 'behavior_trees', nav_to_pose_bt_xml_file))

    nav_through_poses_bt_xml_file = "clean_test.xml"
    nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml',
                                 default=os.path.join(package_prefix, 'behavior_trees', nav_through_poses_bt_xml_file))

    rviz_config_file = os.path.join(package_prefix, 'rviz', 'platform_nav2.rviz')
    pkg_launch_file_dir = os.path.join(package_prefix, 'launch')
    nav2_bringup_launch_file = os.path.join(pkg_launch_file_dir, 'bringup_launch.py')

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file),
        launch_arguments={
            'map': map_dir,
            'params_file': param_dir,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
        }.items(),
    )

    rviz_node = Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen')

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_node)

    return ld