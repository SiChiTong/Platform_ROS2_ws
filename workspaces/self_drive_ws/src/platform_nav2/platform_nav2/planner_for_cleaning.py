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

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.get_logger().info("Wait for Navigation2...")
    navigator.waitUntilNav2Active()

    rate = rate_mine(10.0)

    while rclpy.ok():
        # TODO: CPP 구현

        navigator.update_map_data(str_robot="base_link")
        map: ProcessedMap = navigator.map
        map_np = map.np_map
        img = cv2.cvtColor(map_np, cv2.COLOR_GRAY2RGB)

        approxes = generate_polygon_countour_np(map_np)
        for approx in approxes:
            img = cv2.drawContours(img, [approx], 0, (0,255,0), 3)
        #
        # approxes.insert(0,np.array([[[737, 260]],
        #
        #                              [[738, 261]],
        #
        #                              [[738, 260]]], dtype=np.int32))
        #

        #
        # # generate the boundary by getting the min/max value of all polygon
        # polygons = [np.squeeze(x) for x in approxes]
        #
        # y_limit_lower = min([pt[1] for pt in polygons[0]])
        # y_limit_upper = max([pt[1] for pt in polygons[0]])
        #
        # x_limit_lower = min([pt[0] for pt in polygons[0]])
        # x_limit_upper = max([pt[0] for pt in polygons[0]])
        #
        # # boundary_basic certex order
        # boundary_basic = [[x_limit_lower, y_limit_lower], [x_limit_upper, y_limit_lower], [x_limit_upper, y_limit_upper], [x_limit_lower, y_limit_upper]]
        #
        # # Among all the polygon cv2 generated, [1:] are the inner obstacles
        # obstacles_basic = polygons[1:]
        # # source_basic, dest_basic = [[10, 10], [1000, 500]]
        #
        # boundary, sorted_vertices, obstacles = extract_vertex(boundary_basic, obstacles_basic)
        # draw_problem(boundary, obstacles)
        # open_line_segments = get_vertical_line(sorted_vertices, obstacles,  y_limit_lower, y_limit_upper)

        cv2.imshow("asd",img)

        k = cv2.waitKey(1) & 0xFF
        rclpy.spin_once(navigator, timeout_sec=0.1)
        rate.sleep()

        if k == 27:
            break


if __name__ == '__main__':
    main()
