#!/usr/bin/env python3

from tf2_ros import Buffer,TransformListener

import rclpy
from rclpy.time import Time
from rclpy.node import Node

import csv

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('test')

        self.init_ROS()
        self.timer_period = 0.005  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback, clock=self.get_clock())

    # ROS2 초기화
    def init_ROS(self):
        # TF 송수신용 객체 정의(리소스 할당)
        # TF: 프레임의 자세, 위치 등을 정의하기 위한 메세지 컨테이너 포맷

        self.csv_file = open('/home/park/Desktop/aaa.csv','w',newline='')
        self.wr = csv.writer(self.csv_file)

        self.wr.writerow(['robot_base.x','robot_base.y',
                          # 'robot_base_encoder.x', 'robot_base_encoder.y',
                          # 'robot_base_t265.x', 'robot_base_t265.y',
                          # 'robot_base_no_amcl.x','robot_base_no_amcl.y'
                          ])

        self.buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.buffer, self)

    def timer_callback(self):
        try:
            tf_amcl = self.buffer.lookup_transform("map", "robot_base", Time())  # Blocking
            # tf_dr = self.buffer.lookup_transform("map", "robot_base_encoder", Time())  # Blocking
            # tf_t265 = self.buffer.lookup_transform("map", "robot_base_t265", Time())  # Blocking
            # tf_no_amcl = self.buffer.lookup_transform("map", "robot_base_no_amcl", Time())  # Blocking

            self.wr.writerow([tf_amcl.transform.translation.x,tf_amcl.transform.translation.y,
                              # tf_dr.transform.translation.x,tf_dr.transform.translation.y,
                              # tf_t265.transform.translation.x,tf_t265.transform.translation.y,
                              # tf_no_amcl.transform.translation.x,tf_no_amcl.transform.translation.y,
                              ])

            print(f"{tf_amcl.transform.translation.x},")
                  # f" {tf_dr.transform.translation.x}, "
                  # f"{tf_t265.transform.translation.x}, {tf_no_amcl.transform.translation.x}")

        except Exception as e:
            print(e)

if __name__ == "__main__":
    rclpy.init()
    node = TFListenerNode()
    try:
        rclpy.spin(node)
    except Exception:
        node.csv_file.close()


