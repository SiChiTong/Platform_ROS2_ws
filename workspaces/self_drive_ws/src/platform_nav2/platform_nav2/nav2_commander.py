#! /usr/bin/env python3

from datetime import datetime

import time
import yaml
import os

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from rcl_interfaces.srv import GetParameters
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
from std_msgs.msg import Int8, String
import cv_bridge
from enum import Enum
from action_msgs.msg import GoalStatus

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
                      QoSLivelinessPolicy, QoSReliabilityPolicy

from rclpy.duration import Duration

try:
    from planner_for_cleaning import *
except ModuleNotFoundError:
    from .planner_for_cleaning import *

import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Highly recommended to use platformNavigator method in Main loop only.
# If use these in callback, It may be stuck in infinite loop.

class PlatformController(Enum):
    FollowPath = "FollowPath"
    Cleaning = "Cleaning"

class PlatformGoalChecker(Enum):
    general_goal_checker = "general_goal_checker"
    cleaning_goal_checker = "cleaning_goal_checker"

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
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
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
        self.param_srv = self.create_client(GetParameters, '/map_server/get_parameters')

        self.declare_parameter('robot_base',"base_link")

        self.declare_parameter('visualization_map',False)
        self.declare_parameter('visualization_bbox',True)

        self.robot_base = self.get_parameter('robot_base').value
        self.isVisualization_map = self.get_parameter('visualization_map').value
        self.isVisualization_bbox = self.get_parameter('visualization_bbox').value

        self.info(f"map: {self.isVisualization_map}")
        self.info(f"bbox: {self.isVisualization_bbox}")

        self.waypoint_target = None

        self.get_maps_srv = self.create_client(GetMap, '/map_server/map')

        self.buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.buffer, self)
        self.pub_map_img = self.create_publisher(Image, '/processed_map',1)

        custom_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            avoid_ros_namespace_conventions=False
        )

        self.pub_controller = self.create_publisher(String, '/selected_controller', custom_profile)
        self.pub_planner = self.create_publisher(String, '/selected_planner', custom_profile)
        self.pub_goal_checker = self.create_publisher(String, '/selected_goal_checker', custom_profile)

        self.sub_waypoint = self.create_subscription(Point32, "/waypoint", self.callback_waypoint, 5)
        self.sub_bbox = self.create_subscription(Polygon, "/selected_area", self.callback_bbox, 5)
        self.sub_cmd_gui = self.create_subscription(String, "/cmd_navigator", self.callback_gui, 5)

        self.map = None
        self.cost_map = ProcessedMap(Costmap())

        self.isCleaning = False
        self.flag_stop_cleaning = False
        self.current_progress_path = -1
        self.current_progress_pose = -1

    def waitUntilNav2Active(self):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def isNavComplete(self):
        """Check if the navigation request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.01)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Goal with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True

    def get_map_ros2(self):
        req_param = GetParameters.Request()
        req_param.names = ["yaml_filename"]
        result_future_param = self.param_srv.call_async(req_param)
        rclpy.spin_until_future_complete(self, result_future_param)
        map_yaml_path = result_future_param.result().values[0].string_value

        with open(map_yaml_path) as f:
            map_param = yaml.load(f,Loader=yaml.FullLoader)
            map_split: list = map_yaml_path.split("/")
            del map_split[-1], map_split[0]
            map_param['image'] = '/' + os.path.join(*map_split, map_param['image'])

        return map_param

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

    def set_controller(self, controller: PlatformController):
        result = String()
        result.data = controller.value
        self.pub_controller.publish(result)
        self.pub_planner.publish(result)

        if controller == PlatformController.FollowPath:
            result = String()
            result.data = PlatformGoalChecker.general_goal_checker.value
            self.pub_goal_checker.publish(result)
        elif controller == PlatformController.Cleaning:
            result = String()
            result.data = PlatformGoalChecker.cleaning_goal_checker.value
            self.pub_goal_checker.publish(result)

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

    def callback_waypoint(self, msg: Point32):
        # x, y, z = x, y, yaw
        waypoint_pose = create_pose_from_x_y_yaw(msg.x, msg.y, msg.z, clock=self.get_clock())
        self.waypoint_target = waypoint_pose

    def callback_bbox(self, msg: Polygon):
        if self.check_map():
            if self.isCleaning: self.stop_cleaning()

            if len(msg.points) == 0:
                self.map.set_bbox(None)
                self.info("bbox is initialized!")
            else:
                points = [[int(p.x), int(p.y)] for p in msg.points]
                self.map.set_bbox(points, ratio_x=2.0, ratio_y=2.0)
                self.info(str(points))

    def callback_gui(self, msg: String):
        s = msg.data.split(":")
        category = s[0]
        cmd = s[1]
        if category == "Cleaning":
            if cmd == "Start":
                self.stop_cleaning()
                if self.check_map() and self.map.check_cleaning_path():
                    self.isCleaning = True
                    self.info("get cleaning command from gui")
                else:
                    self.info("get cleaning command from gui... but cleaning path is not calculated!")
                    return
            elif cmd == "Stop":
                self.stop_cleaning()

    def start_cleaning(self):
        if self.isCleaning:
            if(self.current_progress_path == -1):
                self.cancelNav()
                self.cleaning_poses = self.map.get_cleaning_path(self.get_clock())
                self.current_progress_path = 0
                self.current_progress_pose = 0
                self.goToPose(self.cleaning_poses[0][0])

            else:
                print("cleaning command is already progressed.")
        else:
            print("cleaning command is not arrived.")

    def progress_cleaning(self):
        if self.current_progress_path == -1:
            self.info("cleaning process is not started!")
            return False

        if self.isCleaning and self.check_progress_cleaning():
            progressed_path: list = self.cleaning_poses[self.current_progress_path]

            if (self.current_progress_path >= (len(self.cleaning_poses) - 1)) and \
               (self.current_progress_pose >= (len(progressed_path) - 1)):
                self.info("progress all path!")
                self.stop_cleaning()
                return True

            else:
                progressed_path: list = self.cleaning_poses[self.current_progress_path]
                if self.current_progress_pose < (len(progressed_path) - 1):
                    self.current_progress_pose += 1
                    self.set_controller(PlatformController.Cleaning)
                    self.goToPose(progressed_path[self.current_progress_pose])
                else:
                    self.current_progress_path += 1

        return False

    def stop_cleaning(self):
        if self.isCleaning:
            self.current_progress_path = -1
            self.cleaning_poses = None
            self.isCleaning = False
            self.flag_stop_cleaning = True
            self.set_controller(PlatformController.FollowPath)
        else:
            print("cleaning process is already stopped.")

    def check_progress_cleaning(self):
        return hasattr(self, "cleaning_poses") and (self.cleaning_poses is not None) and \
               (self.current_progress_path != -1)

    def check_map(self):
        return hasattr(self, 'map') and (self.map is not None)

class ProcessedMap():
    def __init__(self, map, ratio_x=1.0, ratio_y=1.0, robot_size_x=0.6, robot_size_y=0.4):
        if type(map) == OccupancyGrid:
            size_x = int(map.info.width)
            size_y = int(map.info.height)
            resolution = map.info.resolution
            origin = map.info.origin
            np_map : np.ndarray = np.array(map.data).reshape([size_y, size_x]).astype(np.uint8)
            np_map[np.isin(np_map, [100])] = 255
            np_map = np.flip(np_map,0)

        elif type(map) == Costmap:
            size_x = map.metadata.size_x
            size_y = map.metadata.size_y
            resolution = map.metadata.resolution
            origin = map.metadata.origin
            np_map = np.array(map.data).reshape([size_y, size_x])
            np_map = np.flip(np_map,0)

        elif type(map) == dict:
            np_map = cv2.imread(map['image'])
            np_map = cv2.cvtColor(np_map,cv2.COLOR_BGR2GRAY)
            size_x = np_map.shape[1]
            size_y = np_map.shape[0]
            resolution = map["resolution"]
            origin_list = map["origin"]
            origin = Pose()
            origin.position.x = origin_list[0]
            origin.position.y = origin_list[1]

        else:
            print("Not supported Type!")
            return

        self.np_map = np_map
        self.origin: Pose = origin
        self.resolution = resolution
        self.robot_base = Pose()
        self.size_x = size_x
        self.size_y = size_y
        self.robot_size_x = robot_size_x
        self.robot_size_y = robot_size_y
        self.ratio_x = ratio_x
        self.ratio_y = ratio_y
        self.try_to_calculate_path = False

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
        self.try_to_calculate_path = False

    def get_robot_base(self):
        _, _, yaw = euler_from_quaternion(self.robot_base.orientation.x, self.robot_base.orientation.y,
                                          self.robot_base.orientation.z, self.robot_base.orientation.w)
        x = self.robot_base.position.x
        y = self.robot_base.position.y
        return x, y, yaw

    def get_robot_rect(self, robot_x, robot_y, robot_yaw):
        robot_real = Rectangle(robot_x, robot_y, self.robot_size_y, self.robot_size_x)

        p1, p2, p3, p4 = robot_real.rotate_rectangle(round(robot_yaw,1))

        p1_viz = self.position_to_np(p1.x, p1.y)
        p2_viz = self.position_to_np(p2.x, p2.y)
        p3_viz = self.position_to_np(p3.x, p3.y)
        p4_viz = self.position_to_np(p4.x, p4.y)

        return np.array([p1_viz, p2_viz, p3_viz, p4_viz], dtype=np.int32)

    def get_robot_rect_bbox(self, robot_x, robot_y, robot_yaw):
        robot_real = Rectangle(robot_x, robot_y, self.robot_size_y, self.robot_size_x)

        p1, p2, p3, p4 = robot_real.rotate_rectangle(round(robot_yaw,1))

        p1_viz = self.position_to_np_bbox(p1.x, p1.y)
        p2_viz = self.position_to_np_bbox(p2.x, p2.y)
        p3_viz = self.position_to_np_bbox(p3.x, p3.y)
        p4_viz = self.position_to_np_bbox(p4.x, p4.y)

        return np.array([p1_viz, p2_viz, p3_viz, p4_viz], dtype=np.int32)

    def get_map_bbox(self):
        if self.check_bbox():
            p1 = self.bbox[0]
            p2 = self.bbox[1]
            size_x = p2[0] - p1[0]
            size_y = p2[1] - p1[1]

            map_result = self.np_map[p1[1]:p2[1],p1[0]:p2[0]].copy()
            map_result = cv2.resize(map_result, [int(size_x * self.ratio_x_bbox), int(size_y * self.ratio_y_bbox)])
            return map_result
        else:
            return None

    def origin_np(self):
        return int(-self.origin.position.x / self.resolution), int(self.size_y + self.origin.position.y / self.resolution)

    def position_to_np(self, x, y):
        x_np = int(x / self.resolution)
        y_np = int(-y / self.resolution)
        x_ref, y_ref = self.origin_np()

        return (x_ref + x_np), (y_ref + y_np)

    def position_to_np_bbox(self, x, y):
        x_np = int(x / self.resolution)
        y_np = int(-y / self.resolution)

        x_LU, y_LU = self.bbox[0][0], self.bbox[0][1]
        x_ref, y_ref = self.origin_np()

        return (x_ref + x_np - x_LU), (y_ref + y_np - y_LU)

    def np_to_position_map(self, x, y):
        x_ref, y_ref = self.origin_np()
        return (x / self.ratio_x - x_ref) * self.resolution, -(y / self.ratio_y - y_ref) * self.resolution

    def np_to_position_bbox(self, x, y):
        x_ref, y_ref = self.origin_np()
        x_LU, y_LU = self.bbox[0][0], self.bbox[0][1]
        x_bbox_to_map = x_LU + x / self.ratio_x_bbox
        y_bbox_to_map = y_LU + y / self.ratio_y_bbox
        return (x_bbox_to_map / self.ratio_x - x_ref) * self.resolution, -(y_bbox_to_map / self.ratio_y - y_ref) * self.resolution

    def visualization(self):
        np_map = self.np_map.copy()
        np_map = cv2.cvtColor(np_map,cv2.COLOR_GRAY2RGB)

        origin_x_np, origin_y_np = self.origin_np()
        robot_x, robot_y, robot_yaw = self.get_robot_base()
        points_robot_viz = self.get_robot_rect(robot_x, robot_y, robot_yaw)

        np_map = cv2.circle(np_map, [origin_x_np, origin_y_np], 4, color=[0, 0, 255], thickness=-1)

        if hasattr(self,"waypoint") and (self.waypoint is not None):
            w_x, w_y = self.position_to_np(self.waypoint.position.x, self.waypoint.position.y)
            np_map = cv2.circle(np_map, [w_x, w_y], 8, [255, 0, 0], thickness=-1)

        np_map = cv2.fillPoly(np_map, [points_robot_viz], color=[0, 255, 0])
        np_map = cv2.resize(np_map, [int(self.size_x * self.ratio_x), int(self.size_y * self.ratio_y)])

        if self.check_bbox():
            p1 = self.bbox[0]
            p2 = self.bbox[1]
            np_map = cv2.rectangle(np_map, p1, p2,color=[255, 0, 0],thickness=2)

        return np_map

    def visualization_bbox(self,visualize_progress=False):
        if self.check_bbox():
            p1 = self.bbox[0]
            p2 = self.bbox[1]
            size_x = p2[0] - p1[0]
            size_y = p2[1] - p1[1]

            map_origin = self.np_map[p1[1]:p2[1],p1[0]:p2[0]]
            map_result = map_origin.copy()

            if map_result is not None:
                map_result = cv2.cvtColor(map_result,cv2.COLOR_GRAY2RGB)

            robot_x, robot_y, robot_yaw = self.get_robot_base()
            points_robot_viz = self.get_robot_rect_bbox(robot_x, robot_y, robot_yaw)

            map_result = cv2.fillPoly(map_result, [points_robot_viz], color=[0, 255, 0])
            map_result = cv2.resize(map_result, [int(size_x * self.ratio_x_bbox), int(size_y * self.ratio_y_bbox)])

            cleaning_path = self.calculate_cleaning_path(map_origin, visualize_progress=visualize_progress)

            if cleaning_path is not None:
                map_result = visualize_path(map_result,cleaning_path)

            return map_result
        else:
            return None

    def calculate_cleaning_path(self, bbox_origin, visualize_progress=False):

        # try just once in each bbox
        if not self.try_to_calculate_path:
            self.try_to_calculate_path = True
            try:
                bbox_resize = cv2.resize(bbox_origin, [int(bbox_origin.shape[1] * self.ratio_x_bbox),
                                                       int(bbox_origin.shape[0] * self.ratio_y_bbox)])
                self.cleaning_path= find_cleaning_path(bbox_resize, self, visualize=visualize_progress)
            except Exception as e:
                self.cleaning_path = None
                print(e)

        return self.cleaning_path

    def get_cleaning_path(self, clock: rclpy.node.Clock):
        if self.check_cleaning_path():
            try:
                all_path_pose = []

                for path in self.cleaning_path:
                    path_pose = []
                    vec_x = path[1][0] - path[0][0]
                    vec_y = path[1][1] - path[0][1]
                    yaw = - math.atan2(vec_y, vec_x)
                    for x, y in path:
                        px, py = self.np_to_position_bbox(x, y)
                        path_pose.append(create_pose_from_x_y_yaw(px, py, yaw, clock))
                    all_path_pose.append(path_pose)

                return all_path_pose

            except Exception as e:
                print(e)
                return None
        else:
            return None

    def check_cleaning_path(self):
        return hasattr(self,"cleaning_path") and (self.cleaning_path is not None) and \
               self.check_bbox()

    def check_bbox(self):
        return hasattr(self,"bbox") and (self.bbox is not None)

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

def destroyWindow_mine(win_name):
    try:
        cv2.destroyWindow(win_name)
    except Exception as e:
        if not e.err == "NULL guiReceiver (please create a window)":
            print(e)

def print_progress_msg(navigator: platformNavigator, feedback):
    # Go to Cleaning Path...
    if navigator.isCleaning and navigator.check_progress_cleaning():
        current_pose_index = navigator.current_progress_pose + 1
        all_pose_len = len(navigator.cleaning_poses[navigator.current_progress_path])
        current_cell = navigator.current_progress_path + 1
        all_cells = len(navigator.cleaning_poses)
        navigator.info(f'waypoint: {current_pose_index} / {all_pose_len} | cell: {current_cell} / {all_cells}')

    # Go to Waypoint...
    elif not navigator.isCleaning:
        navigator.info('Estimated time of arrival: ' + '{0:.2f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                          + ' seconds.')

def check_stop_cleaning(navigator: platformNavigator):
    if navigator.flag_stop_cleaning:
        print("stop cleaning!")
        navigator.cancelNav()
        navigator.flag_stop_cleaning = False

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

    isGoalProgress = False
    i = 0

    while rclpy.ok():

        check_stop_cleaning(navigator)

        navigator.update_map_data()
        viz_map = navigator.map.visualization()
        msg_map = cvbridge.cv2_to_imgmsg(viz_map)
        navigator.pub_map_img.publish(msg_map)

        if(navigator.isVisualization_map):
            cv2.imshow(map_win_name,viz_map)
            cv2.setMouseCallback(map_win_name,click_callback_map)

        bbox_area = navigator.map.visualization_bbox(visualize_progress=False)

        if(navigator.isVisualization_bbox) & (bbox_area is not None):
            cv2.imshow(bbox_win_name,bbox_area)
            cv2.setMouseCallback(bbox_win_name,click_callback_bbox)

        elif (bbox_area is None):
            try:
                destroyWindow_mine(bbox_win_name)
            except Exception as e:
                if not e.err == "NULL guiReceiver (please create a window)":
                    print(e)

        # Waypoint
        if navigator.waypoint_target != None:
            # if in cleaning process, stop cleaning.
            if navigator.check_progress_cleaning():
                navigator.stop_cleaning()
                print("Waypoint: stop cleaning")

            if is_goal_first:
                is_goal_first = False
            else:
                navigator.cancelNav()

            if(navigator.goToPose(navigator.waypoint_target)):
                navigator.map.set_waypoint_pose(navigator.waypoint_target)
                print("Waypoint: stop cleaning")
                isGoalProgress = True
            else:
                navigator.map.set_waypoint_pose(None)
                isGoalProgress = False

            navigator.waypoint_target = None

        # Cleaning
        elif(navigator.isCleaning) and (navigator.map.check_cleaning_path()) and (not navigator.check_progress_cleaning()):
            navigator.start_cleaning()
            isGoalProgress = True

        if isGoalProgress:
            if not navigator.isNavComplete():
                i += 1
                if i % 10 == 0:
                    feedback = navigator.getFeedback()
                    print_progress_msg(navigator, feedback)

            # Navigation Process Done.
            else:
                result: NavigationResult = navigator.getResult()

                # Waypoint
                if not navigator.isCleaning:
                    if(result == NavigationResult.SUCCEEDED):
                        navigator.info("Goal Reached!")
                    if(result == NavigationResult.CANCELED):
                        navigator.info("Goal Canceled!")
                    if(result == NavigationResult.FAILED):
                        navigator.info("Goal Failed!")
                    isGoalProgress = False

                # Cleaning...
                else:
                    if(result == NavigationResult.SUCCEEDED):
                        if navigator.progress_cleaning():
                            isGoalProgress = False

                    if(result == NavigationResult.CANCELED):
                        navigator.info("Cleaning Canceled!")
                        navigator.stop_cleaning()
                    if(result == NavigationResult.FAILED):
                        navigator.info("Cleaning Failed!")
                        navigator.stop_cleaning()

        k = cv2.waitKey(1) & 0xFF
        rate.sleep(navigator)

        if k == 27:
            break


if __name__ == '__main__':
    main()
