#! /usr/bin/env python3
import math
import rclpy
import rclpy.timer
import numpy as np
import cv2

from datetime import datetime
import time
import yaml
import os

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import TransformStamped, Polygon, Point32
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, String, Float32
import cv_bridge
from enum import Enum
from action_msgs.msg import GoalStatus

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
                      QoSLivelinessPolicy, QoSReliabilityPolicy

from rclpy.duration import Duration
from detection_2d.msg import DetectionResult, BoundingBox3D

try:
    from planner_for_cleaning import *
except ModuleNotFoundError:
    from .planner_for_cleaning import *

try:
    from navigator_tools import *
except ModuleNotFoundError:
    from .navigator_tools import *

################################# README!!! ###############################################
'''
platformNavigator 클래스 메소드는 반드시 메인 루프에서 사용하는 것을 권장함.
callback 내에서 메소드 사용 시 교착 상태(Deadlock)에 빠질 위험성이 있음.
'''
################################# README!!! ###############################################

def main():
    # ROS2 초기화
    rclpy.init()

    # nav2 관리 클래스 초기화
    navigator = platformNavigator(init_x=0.0, init_y=0.0, init_yaw=0.0)
    is_goal_first = True

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.get_logger().info("Wait for Navigation2...")

    # Nav2가 활성화될 때까지 대기
    navigator.waitUntilNav2Active()

    # 메인 루프 관련 클래스 초기화
    cvbridge = cv_bridge.CvBridge()
    rate = rate_mine(20.0)

    # 디버그용 함수 초기화(클릭 시 위치 좌표 출력)
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

    # 주행 모드 변경을 위한 초기 파라미터 불러오기
    navigator.get_default_costmap_param()
    navigator.get_default_speed_param()
    # 주행 모드 변경은 초기 파라미터에 대해 Ratio로 결정됨. (ex: 저속주행 : 초기 속도 * 0.6)

    while rclpy.ok():
        # 청소 종료 명령을 내린 경우 멈춤
        check_stop_cleaning(navigator)

        # 로봇 위치 업데이트
        navigator.update_map_data()

        # 맵 데이터 이미지로 불러오기
        viz_map = navigator.map.visualization(objects=navigator.objects)
        # 맵 이미지 메세지로 변환 및 Publish
        msg_map = cvbridge.cv2_to_imgmsg(viz_map)
        navigator.pub_map_img.publish(msg_map)

        # 맵 이미지 창 띄우기(옵션에서 설정 가능)
        if(navigator.isVisualization_map):
            cv2.imshow(map_win_name,viz_map)
            cv2.setMouseCallback(map_win_name,click_callback_map)

        # 청소 영역 이미지 불러오기
        selected_area_img = navigator.map.visualization_bbox(visualize_progress=False,threshold_last_step=0.4,objects=navigator.objects)

        # 청소 영역 이미지 창 띄우기
        if(navigator.isVisualization_bbox) & (selected_area_img is not None):
            cv2.imshow(bbox_win_name,selected_area_img)
            cv2.setMouseCallback(bbox_win_name,click_callback_bbox)
        # 청소 영역 초기화 시 창 지우기
        elif (selected_area_img is None):
            try:
                destroyWindow_mine(bbox_win_name)
            except Exception as e:
                if not e.err == "NULL guiReceiver (please create a window)":
                    print(e)

        # 객체가 인식되고 있는 경우
        if (navigator.objects is not None) and (navigator.objects.__len__() > 0):
            from typing import Iterable, cast

            # 객체 중에서 사람에 해당하는 리스트 추출
            objects: list = navigator.objects
            person_list = [a for a in cast(Iterable[BoundingBox3D], objects) if a.name == "person"]

            # 웨이포인트 주행인 경우
            if not navigator.check_progress_cleaning():
                # person_list 중에서 3.0m 이내에 존재하는 리스트 추출
                is_slow = [a for a in cast(Iterable[BoundingBox3D], person_list)
                           if (a.pose.position.x < 3.0) and (math.fabs(a.pose.position.y) < 0.5)]
                # person_list 중에서 1.0m 이내에 존재하는 리스트 추출우
                is_stop = [a for a in cast(Iterable[BoundingBox3D], person_list)
                           if (a.pose.position.x < 1.0) and (math.fabs(a.pose.position.y) < 0.5)]

                # is_stop 리스트에 객체가 존재하는 경우(1m 이내에 사람이 존재하는 경우)
                if (is_stop):
                    # 정지 모드
                    navigator.set_robot_speed(PlatformController.FollowPath,
                                              0.0, 0.0, 0.0)
                # is_slow 리스트에 객체가 존재하는 경우(3m 이내에 사람이 존재하는 경우)
                elif (is_slow):
                    # 저속 주행 모드
                    navigator.set_robot_speed(PlatformController.FollowPath,
                                              navigator.follow_path_max_vel_x_default * 0.6,
                                              0.0,
                                              navigator.follow_path_max_vel_theta_default * 0.6)
                # is_stop, is_slow 모두 해당하지 않는 경우
                else:
                    # 웨이포인트 기본 주행 모드
                    navigator.set_robot_speed(PlatformController.FollowPath,
                                              navigator.follow_path_max_vel_x_default,
                                              0.0,
                                              navigator.follow_path_max_vel_theta_default)
            # 청소 모드인 경우
            else:
                # person_list 중에서 1.0m 이내에 존재하는 리스트 추출
                is_stop = [a for a in cast(Iterable[BoundingBox3D], person_list)
                           if (a.pose.position.x < 1.0) and (math.fabs(a.pose.position.y) < 0.5)]

                # is_stop 리스트에 객체가 존재하는 경우(1m 이내에 사람이 존재하는 경우)
                if (is_stop):
                    # 정지 모드
                    navigator.set_robot_speed(PlatformController.Cleaning,
                                              0.0, 0.0, 0.0)
                # is_stop에 해당하지 않는 경우
                else:
                    # 청소 기본 주행 모드
                    navigator.set_robot_speed(PlatformController.Cleaning,
                                              navigator.cleaning_max_vel_x_default,
                                              navigator.cleaning_max_vel_y_default,
                                              navigator.cleaning_max_vel_theta_default)

        # 객체 인식 기능이 켜지지 않았거나 아무것도 인식되지 않는 경우
        else:
            # 웨이포인트 모드
            if not navigator.check_progress_cleaning():
                # 웨이포인트 기본 주행 모드
                navigator.set_robot_speed(PlatformController.FollowPath,
                                          navigator.follow_path_max_vel_x_default,
                                          0.0,
                                          navigator.follow_path_max_vel_theta_default)
            # 청소 모드
            else:
                # 청소 기본 주행 모드
                navigator.set_robot_speed(PlatformController.Cleaning,
                                          navigator.cleaning_max_vel_x_default,
                                          navigator.cleaning_max_vel_y_default,
                                          navigator.cleaning_max_vel_theta_default)

        # Waypoint
        if navigator.waypoint_target != None:
            # if in cleaning progress, stop cleaning.
            if navigator.check_progress_cleaning():
                navigator.stop_cleaning()
                print("Waypoint: stop cleaning")

            # 처음 목표 설정을 하는 경우
            if is_goal_first:
                is_goal_first = False
            # 설정된 목표가 이미 존재하는 경우
            else:
                # 설정된 목표 캔슬
                navigator.cancelNav()

            # 목표 설정이 nav2 상에서 성공적으로 이루어진 경우
            if(navigator.goToPose(navigator.waypoint_target)):
                # 맵 상에서 웨이포인트 표시
                navigator.map.set_waypoint_pose(navigator.waypoint_target)
                print("Waypoint: go to waypoint")
                isGoalProgress = True
            # 목표 설정이 nav2 상에서 실패한 경우
            else:
                # 맵 상에서 웨이포인트 제거
                navigator.map.set_waypoint_pose(None)
                isGoalProgress = False

            # 목표 설정 진행 후 웨이포인트 초기화
            navigator.waypoint_target = None

        # 청소 명령을 받았는지 / 경로 생성이 완료되었는지 / 청소가 진행되고 있지 않은지 확인
        elif(navigator.isCleaning) and (navigator.map.check_cleaning_path()) and (not navigator.check_progress_cleaning()):
            # 청소 시작
            navigator.start_cleaning()
            isGoalProgress = True

        # 청소 / 웨이포인트 명령을 수행 중인 경우
        if isGoalProgress:
            # If Navigation is in Progress...
            if not navigator.isNavComplete():
                i += 1
                if (i % 10 == 0) and (navigator.verbose):
                    feedback = navigator.getFeedback()
                    print_progress_msg(navigator, feedback)

            # If Navigation Process Done...
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

                # Cleaning
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

        # 명령을 수행 중인지 / 청소가 진행중인지 / 청소 경로가 존재하는지 확인
        if isGoalProgress and navigator.check_progress_cleaning() and hasattr(navigator, 'progressed_path'):
            # 청소에서 목표 자세 확인
            cur_pose: PoseStamped = navigator.progressed_path[navigator.current_progress_pose]
            x = cur_pose.pose.orientation.x
            y = cur_pose.pose.orientation.y
            z = cur_pose.pose.orientation.z
            w = cur_pose.pose.orientation.w

            _, _, cur_yaw = euler_from_quaternion(x,y,z,w)
            # 목표 자세 Publish
            navigator.pub_target_yaw.publish(Float32(data=cur_yaw))

        # 제어 주기를 맞춰주기 위해 잠시 멈춤 + ROS2 spin 동작 수행
        rate.sleep(navigator)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

if __name__ == '__main__':
    main()