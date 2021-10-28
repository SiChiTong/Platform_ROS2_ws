#! /usr/bin/env python3

try:
    from nav2_commander import *
except ModuleNotFoundError:
    from .nav2_commander import *
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
from helpers.graph import *
from helpers.geometry import *
from helpers.sweep import *
from helpers.util import *
# from helpers.util_mp import *
import matplotlib.pyplot as plt
from tsp_solver.greedy import solve_tsp
import csv
import scipy.io as sio

def generate_polygon_countour_np(np_map):
    ret, threshold = cv2.threshold(np_map, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    countours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    approxes = []
    for i, cnt in enumerate(countours):
        episilon = 0.01 * cv2.arcLength(cnt, True)
        approxes.append(cv2.approxPolyDP(cnt, episilon, True))
    return approxes

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

        map: ProcessedMap = navigator.map
        bbox_area = map.visualization_bbox()
        if bbox_area is not None:
            cv2.imshow(bbox_win_name,bbox_area)
            cv2.setMouseCallback(bbox_win_name,click_callback_bbox)


        k = cv2.waitKey(1) & 0xFF

        rate.sleep(navigator)

        if k == 27:
            break


if __name__ == '__main__':
    main()
