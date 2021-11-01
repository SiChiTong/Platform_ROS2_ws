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
    return approxes, threshold

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

    def click_callback_bbox(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            map: ProcessedMap = navigator.map
            x_real,y_real = map.np_to_position_bbox(x,y)
            navigator.info(f"x:{x_real:.2f}, y:{y_real:.2f}")

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
        bbox_area: np.ndarray = map.visualization_bbox()

        if (bbox_area is not None):
            bbox_area_origin = bbox_area.copy()
            scale = bbox_area.shape
            ratio_dilate = 0.05
            kernel = np.ones([int(scale[0]*ratio_dilate),int(scale[1]*ratio_dilate)],np.uint8)
            # bbox_area = cv2.dilate(bbox_area, kernel, iterations = 1)
            bbox_area = cv2.erode(bbox_area, kernel, iterations = 1)
            bbox_area = cv2.copyMakeBorder(bbox_area, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=[255])

            approxes, bbox_area_th = generate_polygon_countour_np(bbox_area)
            bbox_area_viz = cv2.cvtColor(bbox_area_origin,cv2.COLOR_GRAY2RGB)
            for approx in approxes:
                bbox_area_viz = cv2.drawContours(bbox_area_viz, [approx], 0, (0,255,0), 5)

            polygons = [np.squeeze(x) for x in approxes]

            y_limit_lower = min([pt[1] for pt in polygons[0]])
            y_limit_upper = max([pt[1] for pt in polygons[0]])

            x_limit_lower = min([pt[0] for pt in polygons[0]])
            x_limit_upper = max([pt[0] for pt in polygons[0]])

            # boundary_basic certex order
            boundary_basic = [[x_limit_lower, y_limit_lower], [x_limit_upper, y_limit_lower], [x_limit_upper, y_limit_upper], [x_limit_lower, y_limit_upper]]

            # Among all the polygon cv2 generated, [1:] are the inner obstacles
            obstacles_basic = polygons[1:]
            # source_basic, dest_basic = [[10, 10], [1000, 500]]

            boundary, sorted_vertices, obstacles = extract_vertex(boundary_basic, obstacles_basic)
            draw_problem(boundary, obstacles)

            open_line_segments = get_vertical_line(sorted_vertices, obstacles,  y_limit_lower, y_limit_upper)
            plt.show(block=True)

            cv2.imshow("asd",bbox_area_th)

        k = cv2.waitKey(1) & 0xFF

        rate.sleep(navigator)

        if k == 27:
            break


if __name__ == '__main__':
    main()
