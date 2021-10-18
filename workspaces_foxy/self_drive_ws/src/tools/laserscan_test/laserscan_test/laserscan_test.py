import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from pynput import keyboard

try:
    from .submodules.tools import *
except ModuleNotFoundError:
    from submodules.tools import *

class LaserscanTestNode(Node):
    def __init__(self):
        super().__init__('laserscan_test')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.br_static = tf2_ros.StaticTransformBroadcaster(self)

        self.listener = keyboard.Listener(
            on_press=self.on_press_teleop,
            on_release=self.on_release_teleop
        )
        self.listener.start()

        self.timer_period = 0.01  # seconds

        self.timer = self.create_timer(self.timer_period, self.timer_callback_test, clock=self.get_clock())

        self.x_init_1 =  0.140
        self.y_init_1 =  0.155
        self.yaw_init_1 = -119.50

        self.x_init_2 = -0.165
        self.y_init_2 = -0.160
        self.yaw_init_2 = 60.10

        self.yaw_1 = self.yaw_init_1
        self.yaw_2 = self.yaw_init_2
        self.x_1 = self.x_init_1
        self.y_1 = self.y_init_1
        self.x_2 = self.x_init_2
        self.y_2 = self.y_init_2

        self.selected_tf = 0

        self.get_logger().info("broadcasting...")

    def timer_callback_test(self):
        self.broadcast_tf2_static()

    def on_press_teleop(self, key):
        isKeyPress = False

        cmd_yaw = 0.0
        cmd_x = 0.0
        cmd_y = 0.0

        try:
            if (key.char == ","):
                cmd_yaw += 0.1
                isKeyPress = True

            if (key.char == "."):
                cmd_yaw -= 0.1
                isKeyPress = True

        except AttributeError:
            if (key.name == "up"):
                cmd_y += 0.005
                isKeyPress = True

            if (key.name == "down"):
                cmd_y -= 0.005
                isKeyPress = True

            if (key.name == "left"):
                cmd_x -= 0.005
                isKeyPress = True

            if (key.name == "right"):
                cmd_x += 0.005
                isKeyPress = True

        if isKeyPress:
            if(cmd_yaw != 0.0):
                if(self.selected_tf == 1): self.yaw_1 += cmd_yaw
                if(self.selected_tf == 2): self.yaw_2 += cmd_yaw

            if(cmd_x != 0.0):
                if(self.selected_tf == 1): self.x_1 += cmd_x
                if(self.selected_tf == 2): self.x_2 += cmd_x

            if(cmd_y != 0.0):
                if(self.selected_tf == 1): self.y_1 += cmd_y
                if(self.selected_tf == 2): self.y_2 += cmd_y

            if(self.selected_tf == 1):
                self.get_logger().info(f"lidar_1 // x : {self.x_1:.3f} / y : {self.y_1:.3f} / yaw : {self.yaw_1:.3f}")

            if(self.selected_tf == 2):
                self.get_logger().info(f"lidar_2 // x : {self.x_2:.3f} / y : {self.y_2:.3f} / yaw : {self.yaw_2:.3f}")

    def on_release_teleop(self, key):
        try:
            if (key.name == "esc"):
                self.get_logger().info("select None")
                self.selected_tf = 0
        except AttributeError:
            if (key.char == ";"):
                self.get_logger().info("select lidar_1")
                self.selected_tf = 1

            if (key.char == "'"):
                self.get_logger().info("select lidar_2")
                self.selected_tf = 2
            return

    def broadcast_tf2_static(self):
        now_stamp = self.get_clock().now().to_msg()

        t1 = TransformStamped()
        t1.header.stamp = now_stamp
        t1.header.frame_id = "robot_base"
        t1.child_frame_id = "lidar_1_raw"

        t1.transform.translation.x = self.x_1
        t1.transform.translation.y = self.y_1
        t1.transform.translation.z = 0.0

        q1 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_1))

        t1.transform.rotation.w = q1[0]
        t1.transform.rotation.x = q1[1]
        t1.transform.rotation.y = q1[2]
        t1.transform.rotation.z = q1[3]

        t2 = TransformStamped()
        t2.header.stamp = now_stamp
        t2.header.frame_id = "robot_base"
        t2.child_frame_id = "lidar_2_raw"

        t2.transform.translation.x = self.x_2
        t2.transform.translation.y = self.y_2
        t2.transform.translation.z = 0.0

        q2 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_2))

        t2.transform.rotation.w = q2[0]
        t2.transform.rotation.x = q2[1]
        t2.transform.rotation.y = q2[2]
        t2.transform.rotation.z = q2[3]

        t3 = TransformStamped()
        t3.header.stamp = now_stamp
        t3.header.frame_id = "robot_base"
        t3.child_frame_id = "lidar_1"

        t3.transform.translation.x = self.x_1
        t3.transform.translation.y = self.y_1
        t3.transform.translation.z = 0.0

        q3 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_1+180.0))

        t3.transform.rotation.w = q3[0]
        t3.transform.rotation.x = q3[1]
        t3.transform.rotation.y = q3[2]
        t3.transform.rotation.z = q3[3]

        t4 = TransformStamped()
        t4.header.stamp = now_stamp
        t4.header.frame_id = "robot_base"
        t4.child_frame_id = "lidar_2"

        t4.transform.translation.x = self.x_2
        t4.transform.translation.y = self.y_2
        t4.transform.translation.z = 0.0

        q4 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_2+180.0))

        t4.transform.rotation.w = q4[0]
        t4.transform.rotation.x = q4[1]
        t4.transform.rotation.y = q4[2]
        t4.transform.rotation.z = q4[3]

        self.br.sendTransform(t1)
        self.br.sendTransform(t2)
        self.br.sendTransform(t3)
        self.br.sendTransform(t4)

def main(args=None):
    rclpy.init(args=args)

    node = LaserscanTestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
