#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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
import time

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
import rclpy
import rclpy.timer
import numpy as np
import cv2
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
import cv_bridge

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    import math

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def create_pose_from_x_y_yaw(x, y, yaw, clock: rclpy.node.Clock):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = clock.now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.w = q[0]
    pose.pose.orientation.x = q[1]
    pose.pose.orientation.x = q[2]
    pose.pose.orientation.x = q[3]
    return pose

class platformNavigator(BasicNavigator):
    def __init__(self):
        super().__init__()

        self.declare_parameter('waypoints',
                               "[[-14.75,-0.5,0.0], [1.25,0.0,0.0], [10.30,0.0,0.0], [29.75,0.05,0.0], [29.75,8.30,0.0]]")

        self.waypoints = []
        try:
            exec("self.waypoints = "+self.get_parameter('waypoints').value)
        except:
            print(f"Can't load waypoints parameter. {self.get_parameter('waypoints').value}")
            self.waypoints = []

        self.get_maps_srv = self.create_client(GetMap, '/map_server/map')
        self.buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.buffer, self)
        self.pub_map_img = self.create_publisher(Image, '/processed_map',1)
        self.sub_waypoint = self.create_subscription(Int8, "/waypoint", self.callback_waypoint, 5)
        self.waypoint_target = None

        self.map = ProcessedMap(OccupancyGrid())
        self.cost_map = ProcessedMap(Costmap())

    def get_map_ros2(self):
        req = GetMap.Request()
        self.result_future_map = self.get_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, self.result_future_map)
        map = self.result_future_map.result().map
        return map

    def waitUntilNav2Active(self):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate('amcl')
        # self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def set_map(self, map: OccupancyGrid, ratio_x=1.0, ratio_y=1.0):
        self.map = ProcessedMap(map, ratio_x=ratio_x, ratio_y=ratio_y)

    def set_cost_map(self, map: Costmap, ratio_x=1.0, ratio_y=1.0):
        self.map = ProcessedMap(map, ratio_x=ratio_x, ratio_y=ratio_y)

    def get_robot_base(self, target:str):
        try:
            tf = self.buffer.lookup_transform("map", target, rclpy.time.Time())  # Blocking
            return tf
        except Exception as e:
            return None

    def callback_waypoint(self, msg: Int8):
        waypoint = self.waypoints[msg.data - 1]
        waypoint_pose = create_pose_from_x_y_yaw(waypoint[0], waypoint[1], waypoint[2], clock=self.get_clock())
        self.waypoint_target = waypoint_pose

class ProcessedMap():
    def __init__(self, map, ratio_x=1.0, ratio_y=1.0):
        if type(map) == OccupancyGrid:
            size_x = int(map.info.width)
            size_y = int(map.info.height)
            resolution = map.info.resolution
            origin = map.info.origin
            np_map : np.ndarray = np.array(map.data).reshape([size_y, size_x]).astype(np.uint8)
            np_map[np.isin(np_map, [100])] = 255

        elif type(map) == Costmap:
            size_x = map.metadata.size_x
            size_y = map.metadata.size_y
            resolution = map.metadata.resolution
            origin = map.metadata.origin
            np_map = np.array(map.data).reshape([size_y, size_x])

        np_map = np.flip(np_map,0)

        self.np_map = np_map
        self.origin: Pose = origin
        self.resolution = resolution
        self.robot_base = Pose()
        self.size_x = size_x
        self.size_y = size_y
        self.ratio_x = ratio_x
        self.ratio_y = ratio_y

        self.set_robot_base(0, 0, 0)

    def set_robot_base(self, x, y, yaw):
        self.robot_base.position.x = float(x)
        self.robot_base.position.y = float(y)

        q = quaternion_from_euler(0, 0, yaw)
        self.robot_base.orientation.w = q[0]
        self.robot_base.orientation.x = q[1]
        self.robot_base.orientation.y = q[2]
        self.robot_base.orientation.z = q[3]

    def set_robot_base_pose(self, pose:Pose):
        self.robot_base = Pose(position=pose.position, orientation=pose.orientation)

    def set_waypoint_pose(self, pose: PoseStamped):
        if pose is None:
            self.waypoint = None
        else:
            self.waypoint = Pose(position=pose.pose.position, orientation=pose.pose.orientation)

    def set_robot_base_tf(self, tf:TransformStamped):
        self.robot_base.position.x = tf.transform.translation.x
        self.robot_base.position.y = tf.transform.translation.y
        self.robot_base.orientation = tf.transform.rotation

    def origin_np(self):
        return int(-self.origin.position.x / self.resolution), int(self.size_y + self.origin.position.y / self.resolution)

    def position_to_np(self, x, y):
        x_np = int(x / self.resolution)
        y_np = int(-y / self.resolution)
        x_ref, y_ref = self.origin_np()

        return (x_ref + x_np), (y_ref + y_np)

    def np_to_position(self, x, y):
        x_ref, y_ref = self.origin_np()
        return (x / self.ratio_x - x_ref) * self.resolution, -(y / self.ratio_y - y_ref) * self.resolution

    def visualization(self):
        np_map = self.np_map.copy()
        np_map = cv2.cvtColor(np_map,cv2.COLOR_GRAY2RGB)

        origin_x_np, origin_y_np = self.origin_np()
        robot_x, robot_y = self.position_to_np(self.robot_base.position.x, self.robot_base.position.y)

        if hasattr(self,"waypoint") and (self.waypoint is not None):
            w_x, w_y = self.position_to_np(self.waypoint.position.x, self.waypoint.position.y)
            np_map = cv2.circle(np_map, [w_x, w_y], 3, [255, 0, 0], thickness=-1)

        np_map = cv2.circle(np_map, [origin_x_np, origin_y_np], 3, [0, 0, 255], thickness=-1)
        np_map = cv2.circle(np_map, [robot_x, robot_y], 3, [0, 255, 0], thickness=-1)

        np_map = cv2.resize(np_map, [int(self.size_x * self.ratio_x), int(self.size_y * self.ratio_y)])
        return np_map

    def find_position(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x_result,y_result = self.np_to_position(x,y)
            print(f'x: {x_result:.2f} / y: {y_result:.2f}')

def main():
    rclpy.init()

    navigator = platformNavigator()
    is_goal_first = True

    # Set our demo's initial pose
    # initial_pose = create_pose_from_x_y_yaw(0., 0., 0., navigator.get_clock())
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.get_logger().info("Wait for Navigation2...")
    navigator.waitUntilNav2Active()
    cvbridge = cv_bridge.CvBridge()

    map = ProcessedMap(navigator.get_map_ros2(), 0.8, 0.8)
    # global_costmap = ProcessedMap(navigator.getGlobalCostmap(), 1.5, 1.5)
    # global_costmap.set_robot_base_tf(robot_tf)

    while rclpy.ok():
        robot_tf = navigator.get_robot_base("base_link")

        if robot_tf is not None:
            map.set_robot_base_tf(robot_tf)
        # TODO: CPP 구현

        viz_map = map.visualization()
        msg_map = cvbridge.cv2_to_imgmsg(viz_map)
        navigator.pub_map_img.publish(msg_map)

        if navigator.waypoint_target != None:
            if is_goal_first:
                is_goal_first = False
            else:
                navigator.cancelNav()

            navigator.goToPose(navigator.waypoint_target)
            map.set_waypoint_pose(navigator.waypoint_target)
            navigator.waypoint_target = None
            navigator.get_logger().info("Running...")

        k = cv2.waitKey(1) & 0xFF
        time.sleep(0.05)
        rclpy.spin_once(navigator, timeout_sec=0.5)

        if k == 27:
            break


if __name__ == '__main__':
    main()
