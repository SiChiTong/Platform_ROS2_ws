from serial.serialposix import Serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyrealsense2 as rs

from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros
from tools import platform_mode, get_str_command, find_mbed_port, set_bt_port, quaternion_from_euler, deg_to_rad

import serial
from pynput import keyboard
import math

sudoPassword = 'as449856'

# PORT_bt = set_bt_port(sudoPassword)
PORT_mbed = find_mbed_port(sudoPassword)


class PlatformControlNode(Node):
    def __init__(self, ser: Serial):
        super().__init__('localization_node')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.br_static = tf2_ros.StaticTransformBroadcaster(self)

        self.ser = ser

        self.isteleop = False

        self.x_t265 = 0.0
        self.y_t265 = 0.0

        self.x_encoder = 0.0
        self.y_encoder = 0.0

        ctx = rs.context()
        devices = ctx.query_devices()
        devices_dic = {a.get_info(rs.camera_info.name):
                       a.get_info(rs.camera_info.serial_number)
                       for a in devices}

        # Intel RealSense T265
        try:
            self.pipe_t265 = rs.pipeline()
            self.cfg_t265 = rs.config()
            self.cfg_t265.enable_device(devices_dic['Intel RealSense T265'])
            self.cfg_t265.enable_stream(rs.stream.pose)
            self.pipe_t265.start(self.cfg_t265)
        except:
            print("Can't load Intel RealSense T265!")
            self.pipe_t265 = None
            # self.pipe_t265 = None

        self.listener = keyboard.Listener(
            on_press=self.on_press_teleop, on_release=self.on_release_teleop)
        self.listener.start()

        self.yaw_init = None

        self.publisher_result = self.create_publisher(String, 'result', 10)
        self.publisher_tf = self.create_publisher(TFMessage, "/tf", 10)
        self.timer_period = 0.01  # seconds

        self.cur_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(self.timer_period, self.timer_callback, clock=self.get_clock())

    def timer_callback(self):
        try:
            str_msg = self.ser.read_until(b'\n').decode()

            if self.pipe_t265 is not None:
                vx_t265, vy_t265, x_t265, y_t265 = self.get_pose_t265()

            if ('<' in str_msg) & ('>' in str_msg):
                str_1 = str_msg.split('<')
                str_inner = str_1[str_1.__len__() - 1].split('>')[0]
                str_dict = str_inner.split(';')
                str_dict.remove('')
                dict_str = {a.split(':')[0]: float(a.split(':')[1])
                            for a in str_dict}

                if self.yaw_init is None:
                    self.yaw_init = dict_str['Y']
                    self.prev_time = self.get_clock().now()

                dict_str['Y'] = dict_str['Y'] - self.yaw_init

                yaw_rad = dict_str['Y'] / 180.0 * math.pi

                if self.pipe_t265 is not None:
                    self.calculate_x_y(dict_str['VX'], dict_str['VY'], vx_t265, vy_t265, yaw_rad)
                    # print('t265 : {0:.3f}  {1:.3f}'.format(x_t265, y_t265))

                    self.broadcast_tf2(x_t265, y_t265, self.x_encoder, self.y_encoder, yaw_rad)
        except:
            self.ser.reset_input_buffer()
            return

    def get_pose_t265(self):
        frames = self.pipe_t265.wait_for_frames()
        pose = frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
            return -data.velocity.z, -data.velocity.x, -data.translation.z, -data.translation.x

    def on_press_teleop(self, key):
        if (self.isteleop):
            try:
                if (key.char == "w"):
                    str_cmd = get_str_command(platform_mode.Front, 20.0)
                if (key.char == "s"):
                    str_cmd = get_str_command(platform_mode.Back, 20.0)
                if (key.char == "a"):
                    str_cmd = get_str_command(platform_mode.Left, 20.0)
                if (key.char == "d"):
                    str_cmd = get_str_command(platform_mode.Right, 20.0)
                if (key.char == "q"):
                    str_cmd = get_str_command(platform_mode.TLeft, 20.0)
                if (key.char == "e"):
                    str_cmd = get_str_command(platform_mode.TRight, 20.0)

            except AttributeError:
                if (key.name == "space"):
                    str_cmd = get_str_command(platform_mode.Stop, 0)

            try:
                self.ser.write(str_cmd.encode())
            except:
                return

    def on_release_teleop(self, key):
        try:
            if (key.name == "f2"):
                if (not self.isteleop):
                    print("Teleop mode ON!")
                    self.isteleop = True
                else:
                    print("Teleop mode OFF!")
                    self.isteleop = False
        except:
            return

    def broadcast_tf2(self, x_t265, y_t265, x_encoder, y_encoder, yaw_rad):
        # offset = 60000000
        offset = 30000000
        now_stamp = self.get_clock().now().to_msg()
        # now_stamp.nanosec += offset

        t1 = TransformStamped()
        t1.header.stamp = now_stamp
        t1.header.frame_id = "odom"
        t1.child_frame_id = "robot_base"
        t1.transform.translation.x = x_t265
        t1.transform.translation.y = y_t265
        t1.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, yaw_rad)

        t1.transform.rotation.w = q[0]
        t1.transform.rotation.x = q[1]
        t1.transform.rotation.y = q[2]
        t1.transform.rotation.z = q[3]

        t2 = TransformStamped()
        t2.header.stamp = now_stamp
        t2.header.frame_id = "odom"
        t2.child_frame_id = "robot_base_encoder"
        t2.transform.translation.x = x_encoder
        t2.transform.translation.y = y_encoder
        t2.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, yaw_rad)

        t2.transform.rotation.w = q[0]
        t2.transform.rotation.x = q[1]
        t2.transform.rotation.y = q[2]
        t2.transform.rotation.z = q[3]

        self.broadcast_tf2_lidar()

        self.br.sendTransform(t1)
        self.br.sendTransform(t2)

    def calculate_x_y(self, vx_encoder, vy_encoder, vx_t265, vy_t265, yaw_rad):
        self.cur_time = self.get_clock().now()

        dt = (self.cur_time - self.prev_time).nanoseconds / 1000000000.0
        self.prev_time = self.get_clock().now()

        #  # vy_encoder *= vy_offset

        self.x_encoder += vx_encoder * math.cos(yaw_rad) * dt
        self.x_encoder += vy_encoder * math.sin(yaw_rad) * dt
        self.y_encoder += vx_encoder * math.sin(yaw_rad) * dt
        self.y_encoder += vy_encoder * math.cos(yaw_rad) * dt

        self.x_t265 += vx_t265 * math.cos(yaw_rad) * dt
        self.x_t265 += vy_t265 * math.sin(yaw_rad) * dt
        self.y_t265 += vx_t265 * math.sin(yaw_rad) * dt
        self.y_t265 += vy_t265 * math.cos(yaw_rad) * dt

        print('encoder : {0:.3f}  {1:.3f}\nt265 : {2:.3f}  {3:.3f}'.format(
              vx_encoder, vy_encoder, vx_t265, vy_t265))

        # print('encoder : {0:.3f}  {1:.3f}\nt265 : {2:.3f}  {3:.3f}'.format(
        #     self.x_encoder, self.y_encoder, self.x_t265, self.y_t265))

        #print('encoder : {0:.3f}  {1:.3f}'.format(
        #    self.x_encoder, self.y_encoder))

    def broadcast_tf2_lidar(self):
        now_stamp = self.get_clock().now().to_msg()

        t1 = TransformStamped()
        t1.header.stamp = now_stamp
        t1.header.frame_id = "robot_base"
        t1.child_frame_id = "lidar_1"
        t1.transform.translation.x = 0.160
        t1.transform.translation.y = 0.150
        t1.transform.translation.z = 0.0

        q1 = quaternion_from_euler(0, 0, deg_to_rad(5))

        t1.transform.rotation.w = q1[0]
        t1.transform.rotation.x = q1[1]
        t1.transform.rotation.y = q1[2]
        t1.transform.rotation.z = q1[3]

        t2 = TransformStamped()
        t2.header.stamp = now_stamp
        t2.header.frame_id = "robot_base"
        t2.child_frame_id = "lidar_2"
        t2.transform.translation.x = -0.160
        t2.transform.translation.y = -0.150
        t2.transform.translation.z = 0.0

        q2 = quaternion_from_euler(0, 0, deg_to_rad(195))

        t2.transform.rotation.w = q2[0]
        t2.transform.rotation.x = q2[1]
        t2.transform.rotation.y = q2[2]
        t2.transform.rotation.z = q2[3]

        self.br_static.sendTransform(t1)
        self.br_static.sendTransform(t2)

def main(args=None):
    rclpy.init(args=args)
    # 포트 설정
    PORT = PORT_mbed
    # 연결
    ser = serial.serial_for_url(PORT, baudrate=115200, timeout=1)

    bt_pub = PlatformControlNode(ser)

    rclpy.spin(bt_pub)

    bt_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
