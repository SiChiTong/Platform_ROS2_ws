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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-1.5')

    turtlebot_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    map_file = "map_house.yaml"
    map_dir = LaunchConfiguration('map_file',
                                  default=os.path.join(package_prefix, 'map', map_file))

    param_file_name = 'nav2_sim.yaml'
    param_dir = LaunchConfiguration('params_file',
                                    default=os.path.join(package_prefix,'config',param_file_name))

    nav_to_pose_bt_xml_file = "nav_test.xml"
    nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml',
                                 default=os.path.join(package_prefix, 'behavior_trees', nav_to_pose_bt_xml_file))

    nav_through_poses_bt_xml_file = "clean_test.xml"
    nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml',
                                 default=os.path.join(package_prefix, 'behavior_trees', nav_through_poses_bt_xml_file))

    pkg_launch_file_dir = os.path.join(package_prefix, 'launch')
    rviz_config_file = os.path.join(package_prefix, 'rviz', 'platform_sim.rviz')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_launch_file_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
        }.items(),
    )

    gui_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_prefix, 'launch', 'nav2_gui.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    rviz_node = Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen')

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_node)

    ld.add_action(gui_cmd)

    return ld