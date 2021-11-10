#! /usr/bin/env python3

try:
    from helpers.graph import *
    from helpers.geometry import *
    from helpers.sweep import *
    from helpers.util import *
except ModuleNotFoundError:
    from .helpers.graph import *
    from .helpers.geometry import *
    from .helpers.sweep import *
    from .helpers.util import *

import matplotlib.pyplot as plt

from tsp_solver.greedy import solve_tsp
import csv
import scipy.io as sio

def generate_polygon_countour_np(np_map):
    # ret, threshold = cv2.threshold(np_map, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ret, threshold = cv2.threshold(np_map, 250, 255, cv2.THRESH_BINARY)
    countours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    approxes = []
    for i, cnt in enumerate(countours):
        episilon = 0.002 * cv2.arcLength(cnt, True)
        approxes.append(cv2.approxPolyDP(cnt, episilon, True))
    return approxes, threshold

def visualize_contours(bbox_area_origin, approxes):
    bbox_area_viz_cont = bbox_area_origin.copy()
    bbox_area_viz_cont = cv2.copyMakeBorder(bbox_area_viz_cont, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=[255, 255, 255])
    bbox_area_viz_cont = cv2.cvtColor(bbox_area_viz_cont, cv2.COLOR_GRAY2RGB)

    for approx in approxes[1:]:
        # bbox_area_viz_cont = cv2.drawContours(bbox_area_viz_cont, [approx], 0, (0,255,0), 5)
        bbox_area_viz_cont = cv2.drawContours(bbox_area_viz_cont, [approx], 0, np.random.randint(0,255,3).tolist(), -1)

    return bbox_area_viz_cont

def visualize_path(bbox_area_origin, path_nodes):
    bbox_area_viz_path = bbox_area_origin.copy()
    if len(bbox_area_viz_path.shape) == 2:
        bbox_area_viz_path = cv2.cvtColor(bbox_area_viz_path, cv2.COLOR_GRAY2RGB)

    full_path=[]
    for path_node in path_nodes: full_path.append(np.array(path_node))

    bbox_area_viz_path = cv2.polylines(bbox_area_viz_path, full_path, isClosed=False, color=[0, 255, 0], thickness=2)

    for path in full_path:
        if len(path) >= 2:
            bbox_area_viz_path = cv2.circle(bbox_area_viz_path, path[0], 3, [0, 0, 255], thickness=-1)
            bbox_area_viz_path = cv2.circle(bbox_area_viz_path, path[-1], 3, [255, 0, 0], thickness=-1)

    return bbox_area_viz_path

def remove_too_small_cells(quad_cells: list, robot_w, robot_h):
    for cell in quad_cells[:]:
        cell_w = cell[1].x - cell[0].x
        cell_h = cell[2].y - cell[1].y

        isSmall = (robot_h > cell_w) | (robot_h > cell_h)
        if isSmall:
            quad_cells.remove(cell)

def find_cleaning_path(bbox_area, map, visualize=False):
    if (bbox_area is not None):

        bbox_area_origin = bbox_area.copy()
        bbox_area_viz = bbox_area.copy()

        scale = bbox_area_viz.shape
        ratio_dilate = 0.03
        kernel = np.ones([int(scale[0]*ratio_dilate),int(scale[1]*ratio_dilate)],np.uint8)

        bbox_area_viz = cv2.morphologyEx(bbox_area_viz, cv2.MORPH_CLOSE, np.ones([30, 30], np.uint8), iterations=1)
        # bbox_area = cv2.erode(bbox_area, kernel, iterations = 1)
        bbox_area_viz = cv2.copyMakeBorder(bbox_area_viz, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=[255])

        approxes, bbox_area_th = generate_polygon_countour_np(bbox_area_viz)
        path = find_path(map, approxes, visualize=visualize)

        if visualize:
            bbox_area_viz_morp = cv2.cvtColor(bbox_area_th.copy(), cv2.COLOR_GRAY2RGB)
            bbox_area_viz_cont = visualize_contours(bbox_area_origin, approxes)
            bbox_area_result = np.concatenate((bbox_area_viz_morp, bbox_area_viz_cont), axis=0)
            cv2.imshow("asd",bbox_area_result)

        return path

def find_path(map, approxes, visualize=False):
    polygons = [np.squeeze(x) for x in approxes]

    y_limit_lower = min([pt[1] for pt in polygons[0]])
    y_limit_upper = max([pt[1] for pt in polygons[0]])

    x_limit_lower = min([pt[0] for pt in polygons[0]])
    x_limit_upper = max([pt[0] for pt in polygons[0]])

    # boundary_basic certex order
    boundary_basic = [[x_limit_lower, y_limit_lower], [x_limit_upper, y_limit_lower], [x_limit_upper, y_limit_upper], [x_limit_lower, y_limit_upper]]

    # Among all the polygon cv2 generated, [1:] are the inner obstacles
    obstacles_basic = polygons[1:]

    boundary, sorted_vertices, obstacles = extract_vertex(boundary_basic, obstacles_basic)

    # a0 = plt.figure(figsize=(20,10),num=0)
    # draw_problem(boundary, obstacles)
    open_line_segments = get_vertical_line(sorted_vertices, obstacles,  y_limit_lower, y_limit_upper)

    quad_cells, left_tri_cells, right_tri_cells = generate_naive_polygon(open_line_segments, sorted_vertices, obstacles)
    # a1 = plt.figure(figsize=(20,10),num=1)
    # draw_cell(quad_cells, boundary, obstacles)

    robot_footprint = map.get_robot_rect_bbox(0, 0, 0)
    robot_w = robot_footprint[2][1] - robot_footprint[0][1]
    robot_h = robot_footprint[2][0] - robot_footprint[0][0]

    refine_quad_cells(quad_cells)
    remove_too_small_cells(quad_cells, robot_w, robot_h)

    # if(visualize):
    #     a2 = plt.figure(figsize=(20,10),num=2)
    #     draw_cell(quad_cells, boundary, obstacles)

    # ------------------------------------------------------
    # Add boundary lines

    if(len(sorted_vertices) != 0):
        if( boundary[0].x != sorted_vertices[0].x):
            quad_cells.append([boundary[0], point(sorted_vertices[0].x, y_limit_lower), point(sorted_vertices[0].x, y_limit_upper), boundary[3]]);
        if( boundary[1].x != sorted_vertices[len(sorted_vertices)-1].x):
            quad_cells.append([point(sorted_vertices[len(sorted_vertices)-1].x ,y_limit_lower), boundary[1], boundary[2], point(sorted_vertices[len(sorted_vertices)-1].x, y_limit_upper)]);
    else:
        quad_cells = [boundary]

    all_cell = quad_cells + left_tri_cells + right_tri_cells
    # sort the cell based on teh x-value of the first point
    ################-----   IMPORTANT  -----##################
    all_cell.sort(key = lambda pnt: pnt[0].x)

    # if(visualize):
    #     a3 = plt.figure(figsize=(20,10),num=3)
    #     draw_cell(all_cell, boundary, obstacles, fill=True)



    #-------------------------------------------------------
    # Plot final cells

    # nodes = generate_node_set(all_cell, width=robot_w, step=robot_h * 2, safeWidth=robot_w * 2)
    nodes = generate_node_set(all_cell, width=robot_w * 2, safeWidth=robot_w * 2)

    if(visualize): a4 = plt.figure(figsize=(20,10),num=4)

    path = []

    for index, i in enumerate(nodes):
        draw_problem(boundary, obstacles)
        # plt.figure(index)
        x = [j.x for j in i.polygon]
        x.append(x[0])
        y = [j.y for j in i.polygon]
        y.append(y[0])
        plt.plot(x, y)

        path_x = [pnt.x for pnt in i.inside_path]
        path_y = [pnt.y for pnt in i.inside_path]
        path_node = [[int(pnt.x), int(pnt.y)] for pnt in i.inside_path]
        path.append(path_node)

        plt.plot(path_x, path_y)
        for pnt in i.inside_path:
            plt.plot(pnt.x, pnt.y, marker="o")

        center = centroid(i.polygon)
        plt.plot(center.x, center.y, marker="o")
        plt.annotate('cell-{}'.format(index), xy=(center.x, center.y))

    if(visualize): plt.show(block=True)
    return path

# def main():
#     rclpy.init()
#     navigator = platformNavigator(init_x=0.0, init_y=0.0, init_yaw=0.0)
#
#     # executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
#     # aa = executor.add_node(navigator)
#     #
#     # executor_thread = threading.Thread(target=executor.spin, daemon=True)
#     # executor_thread.start()
#
#     is_goal_first = True
#
#     # Wait for navigation to fully activate, since autostarting nav2
#     navigator.get_logger().info("Wait for Navigation2...")
#     navigator.waitUntilNav2Active()
#     cvbridge = cv_bridge.CvBridge()
#
#     rate = rate_mine(30.0)
#
#     while rclpy.ok():
#
#         # TODO: CPP 구현
#         # navigator.update_map_data(str_robot="robot_base")
#
#         navigator.update_map_data()
#         viz_map = navigator.map.visualization()
#         msg_map = cvbridge.cv2_to_imgmsg(viz_map)
#         navigator.pub_map_img.publish(msg_map)
#
#         if navigator.waypoint_target != None:
#             if is_goal_first:
#                 is_goal_first = False
#             else:
#                 navigator.cancelNav()
#             navigator.goToPose(navigator.waypoint_target)
#             navigator.waypoint_target = None
#             navigator.get_logger().info("Running...")
#
#         map: ProcessedMap = navigator.map
#         bbox_area: np.ndarray = map.get_map_bbox()
#
#         path = find_cleaning_path(bbox_area, map,visualize=True)
#
#         k = cv2.waitKey(1) & 0xFF
#
#         rate.sleep(navigator)
#
#         if k == 27:
#             break
#
#
# if __name__ == '__main__':
#     main()
