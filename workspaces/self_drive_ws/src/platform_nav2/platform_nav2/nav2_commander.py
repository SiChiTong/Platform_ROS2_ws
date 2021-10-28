#! /usr/bin/env python3

from datetime import datetime
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
from geometry_msgs.msg import TransformStamped, Polygon, Point32
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
import cv_bridge
import math

import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

def euler_from_quaternion(x, y, z, w):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    # x = quaternion.x
    # y = quaternion.y
    # z = quaternion.z
    # w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

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
    def __init__(self, init_x=0.0, init_y=0.0, init_yaw=0.0):
        super().__init__()

        self.initial_pose = create_pose_from_x_y_yaw(init_x, init_y, init_yaw, self.get_clock())

        self.declare_parameter('waypoints',
                               "[[-14.75,-0.5,0.0], [1.25,0.0,0.0], [10.30,0.0,0.0], [29.75,0.05,0.0], [29.75,8.30,0.0]]")

        self.declare_parameter('robot_base',"robot_base")

        self.declare_parameter('visualization_map',False)
        self.declare_parameter('visualization_bbox',True)

        self.robot_base = self.get_parameter('robot_base').value
        self.isVisualization_map = self.get_parameter('visualization_map').value
        self.isVisualization_bbox = self.get_parameter('visualization_bbox').value

        self.info(f"map: {self.isVisualization_map}")
        self.info(f"bbox: {self.isVisualization_bbox}")

        self.waypoints = []
        self.waypoint_target = None

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
        self.sub_bbox = self.create_subscription(Polygon, "/selected_area", self.callback_bbox, 5)

        self.map = None
        self.cost_map = ProcessedMap(Costmap())

    def waitUntilNav2Active(self):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def get_map_ros2(self):
        req = GetMap.Request()
        self.result_future_map = self.get_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, self.result_future_map)
        map = self.result_future_map.result().map
        return map

    def get_robot_base(self, target:str):
        try:
            tf = self.buffer.lookup_transform("map", target, rclpy.time.Time())  # Blocking
            return tf
        except Exception as e:
            self.info(f"error: {e}")
            return None

    def set_map(self, map: OccupancyGrid, ratio_x=1.0, ratio_y=1.0):
        self.map = ProcessedMap(map, ratio_x=ratio_x, ratio_y=ratio_y)

    def set_cost_map(self, map: Costmap, ratio_x=1.0, ratio_y=1.0):
        self.cost_map = ProcessedMap(map, ratio_x=ratio_x, ratio_y=ratio_y)

    def update_map_data(self, str_robot=None, ratio_x=1.0, ratio_y=1.0):
        if(str_robot is None): str_robot=self.robot_base

        if(self.map is None):
            self.set_map(self.get_map_ros2(), ratio_x=ratio_x, ratio_y=ratio_y)
        # costmap = ProcessedMap(self.getGlobalCostmap())

        robot_tf = self.get_robot_base(str_robot)
        if robot_tf is not None:
            self.map.set_robot_base_tf(robot_tf)
            # costmap.set_robot_base_tf(robot_tf)

        # self.cost_map = costmap

    def callback_waypoint(self, msg: Int8):
        waypoint = self.waypoints[msg.data - 1]
        waypoint_pose = create_pose_from_x_y_yaw(waypoint[0], waypoint[1], waypoint[2], clock=self.get_clock())

        self.waypoint_target = waypoint_pose

        if self.waypoint_target != None:
            self.map.set_waypoint_pose(self.waypoint_target)
            self.cost_map.set_waypoint_pose(self.waypoint_target)

    def callback_bbox(self, msg: Polygon):
        # points = [[x1,y1], [x2,y2]]
        points = [[int(p.x), int(p.y)] for p in msg.points]

        self.map.set_bbox(points, ratio_x=2.0, ratio_y=2.0)
        self.info(str(points))

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

    def set_bbox(self, points, ratio_x=1.0, ratio_y=1.0):
        # points = [[x1,y1], [x2,y2]]
        self.bbox = points
        self.ratio_x_bbox = ratio_x
        self.ratio_y_bbox = ratio_y

    def get_robot_base(self):
        _, _, yaw = euler_from_quaternion(self.robot_base.orientation.x, self.robot_base.orientation.y,
                                          self.robot_base.orientation.z, self.robot_base.orientation.w)
        x = self.robot_base.position.x
        y = self.robot_base.position.y
        return x, y, yaw

    def origin_np(self):
        return int(-self.origin.position.x / self.resolution), int(self.size_y + self.origin.position.y / self.resolution)

    def position_to_np(self, x, y):
        x_np = int(x / self.resolution)
        y_np = int(-y / self.resolution)
        x_ref, y_ref = self.origin_np()

        return (x_ref + x_np), (y_ref + y_np)

    def np_to_position_map(self, x, y):
        x_ref, y_ref = self.origin_np()
        return (x / self.ratio_x - x_ref) * self.resolution, -(y / self.ratio_y - y_ref) * self.resolution

    def np_to_position_bbox(self, x, y):
        x_ref, y_ref = self.origin_np()
        x_LU, y_LU = self.bbox[0][0], self.bbox[0][1]
        x_bbox_to_map = x_LU + x / self.ratio_x_bbox
        y_bbox_to_map = y_LU + y / self.ratio_x_bbox
        return (x_bbox_to_map / self.ratio_x - x_ref) * self.resolution, -(y_bbox_to_map / self.ratio_y - y_ref) * self.resolution

    def visualization(self):
        np_map = self.np_map.copy()
        np_map = cv2.cvtColor(np_map,cv2.COLOR_GRAY2RGB)

        origin_x_np, origin_y_np = self.origin_np()
        robot_x, robot_y, robot_yaw = self.get_robot_base()
        robot_real = Rectangle(robot_x, robot_y, 0.4, 0.6)

        p1, p2, p3, p4 = robot_real.rotate_rectangle(round(robot_yaw,1))

        p1_viz = self.position_to_np(p1.x, p1.y)
        p2_viz = self.position_to_np(p2.x, p2.y)
        p3_viz = self.position_to_np(p3.x, p3.y)
        p4_viz = self.position_to_np(p4.x, p4.y)

        points_robot_viz = np.array([p1_viz, p2_viz, p3_viz, p4_viz], dtype=np.int32)

        if hasattr(self,"waypoint") and (self.waypoint is not None):
            w_x, w_y = self.position_to_np(self.waypoint.position.x, self.waypoint.position.y)
            np_map = cv2.circle(np_map, [w_x, w_y], 3, [255, 0, 0], thickness=-1)

        np_map = cv2.circle(np_map, [origin_x_np, origin_y_np], 3, color=[0, 0, 255], thickness=-1)
        np_map = cv2.fillPoly(np_map, [points_robot_viz], color=[0, 255, 0])
        np_map = cv2.resize(np_map, [int(self.size_x * self.ratio_x), int(self.size_y * self.ratio_y)])

        if hasattr(self,"bbox") and (self.bbox is not None):
            p1 = self.bbox[0]
            p2 = self.bbox[1]
            np_map = cv2.rectangle(np_map, p1, p2,color=[255, 0, 0],thickness=2)

        return np_map

    def visualization_bbox(self):
        if hasattr(self,"bbox") and (self.bbox is not None):
            p1 = self.bbox[0]
            p2 = self.bbox[1]
            size_x = p2[0] - p1[0]
            size_y = p2[1] - p1[1]

            map_result = self.np_map[p1[1]:p2[1],p1[0]:p2[0]]
            map_result = cv2.resize(map_result, [int(size_x * self.ratio_x_bbox), int(size_y * self.ratio_y_bbox)])
            return map_result
        else:
            return None

class rate_mine():
    def __init__(self, rate):
        self.rate = rate
        self.dt0 = datetime.now()

    def sleep(self, node):
        process_time = (datetime.now() - self.dt0).total_seconds()
        period = 1.0 / self.rate

        while (process_time <= period):
            rclpy.spin_once(node, timeout_sec=0.1)
            process_time = (datetime.now() - self.dt0).total_seconds()

        self.dt0 = datetime.now()

class Point:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

class Rectangle:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = h
        self.h = w
        self.angle = 0.0

    def rotate_rectangle(self, theta):
        pt0, pt1, pt2, pt3 = self.get_vertices_points()

        # Point 0
        rotated_x = math.cos(theta) * (pt0.x - self.x) - math.sin(theta) * (pt0.y - self.y) + self.x
        rotated_y = math.sin(theta) * (pt0.x - self.x) + math.cos(theta) * (pt0.y - self.y) + self.y
        point_0 = Point(rotated_x, rotated_y)

        # Point 1
        rotated_x = math.cos(theta) * (pt1.x - self.x) - math.sin(theta) * (pt1.y - self.y) + self.x
        rotated_y = math.sin(theta) * (pt1.x - self.x) + math.cos(theta) * (pt1.y - self.y) + self.y
        point_1 = Point(rotated_x, rotated_y)

        # Point 2
        rotated_x = math.cos(theta) * (pt2.x - self.x) - math.sin(theta) * (pt2.y - self.y) + self.x
        rotated_y = math.sin(theta) * (pt2.x - self.x) + math.cos(theta) * (pt2.y - self.y) + self.y
        point_2 = Point(rotated_x, rotated_y)

        # Point 3
        rotated_x = math.cos(theta) * (pt3.x - self.x) - math.sin(theta) * (pt3.y - self.y) + self.x
        rotated_y = math.sin(theta) * (pt3.x - self.x) + math.cos(theta) * (pt3.y - self.y) + self.y
        point_3 = Point(rotated_x, rotated_y)

        return point_0, point_1, point_2, point_3

    def get_vertices_points(self):
        x0, y0, width, height, _angle = self.x, self.y, self.w, self.h, self.angle
        b = math.cos(math.radians(_angle)) * 0.5
        a = math.sin(math.radians(_angle)) * 0.5
        pt0 = Point(float(x0 - a * height - b * width), float(y0 + b * height - a * width))
        pt1 = Point(float(x0 + a * height - b * width), float(y0 - b * height - a * width))
        pt2 = Point(float(2 * x0 - pt0.x), float(2 * y0 - pt0.y))
        pt3 = Point(float(2 * x0 - pt1.x), float(2 * y0 - pt1.y))
        pts = [pt0, pt1, pt2, pt3]
        return pts

def main():
    rclpy.init()
    navigator = platformNavigator(init_x=0.0, init_y=0.0, init_yaw=0.0)

    # executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    # aa = executor.add_node(navigator)
    #
    # executor_thread = threading.Thread(target=executor.spin, daemon=True)
    # executor_thread.start()

    is_goal_first = True

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.get_logger().info("Wait for Navigation2...")
    navigator.waitUntilNav2Active()
    cvbridge = cv_bridge.CvBridge()

    rate = rate_mine(30.0)

    def click_callback_map(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            map: ProcessedMap = navigator.map
            x_real,y_real = map.np_to_position_map(x,y)
            navigator.info(f"x:{x_real:.2f}, y:{y_real:.2f}")

    def click_callback_bbox(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            map: ProcessedMap = navigator.map
            x_real,y_real = map.np_to_position_bbox(x,y)
            navigator.info(f"x:{x_real:.2f}, y:{y_real:.2f}")

    map_win_name = "map"
    bbox_win_name = "bbox"

    while rclpy.ok():

        # TODO: CPP 구현
        # navigator.update_map_data(str_robot="robot_base")
        navigator.update_map_data()
        viz_map = navigator.map.visualization()
        msg_map = cvbridge.cv2_to_imgmsg(viz_map)
        navigator.pub_map_img.publish(msg_map)

        if navigator.waypoint_target != None:
            if is_goal_first:
                is_goal_first = False
            else:
                navigator.cancelNav()
            navigator.goToPose(navigator.waypoint_target)
            navigator.waypoint_target = None
            navigator.get_logger().info("Running...")

        if(navigator.isVisualization_map):
            cv2.imshow(map_win_name,viz_map)
            cv2.setMouseCallback(map_win_name,click_callback_map)

        map: ProcessedMap = navigator.map
        bbox_area = map.visualization_bbox()

        if (bbox_area is not None) & (navigator.isVisualization_bbox):
            cv2.imshow(bbox_win_name,bbox_area)
            cv2.setMouseCallback(bbox_win_name,click_callback_bbox)

        k = cv2.waitKey(1) & 0xFF

        rate.sleep(navigator)

        if k == 27:
            break


if __name__ == '__main__':
    main()
