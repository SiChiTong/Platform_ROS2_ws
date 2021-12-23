#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

try:
    import pyrealsense2 as rs
except:
    import pyrealsense2.pyrealsense2 as rs

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf2_ros import Buffer,TransformListener

import tf2_ros
import serial
from pynput import keyboard

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
    QoSLivelinessPolicy, QoSReliabilityPolicy

try:
    from submodules.tools import *
except ModuleNotFoundError:
    from .submodules.tools import *

sudoPassword = 'as449856'

class PlatformControlNode(Node):
    # ---------------------------------- init ---------------------------------- #
    def __init__(self, port: str):
        super().__init__('platform_node')

        self.declare_parameter('mode',"NAVIGATION")
        self.ser = self.connect_ser(port)
        self.isRealPlatform = True

        if (self.ser is None):
            self.print_info("Can't Connect to mbed port!")
            self.shutdown_node()

        self.init_ROS()
        self.init_platform()
        self.init_realsense()
        self.init_feedback()
        self.init_teleop()

        self.timer_period = 0.01  # seconds

        self.timer = self.create_timer(self.timer_period,
                                       self.timer_callback,
                                       clock=self.get_clock())

    # 시리얼 포트 초기화
    def connect_ser(self, PORT: str):
        count_connection = 0
        ser = None
        isconnect = False

        while (count_connection < 15) and (not isconnect):
            try:
                # 시리얼 통신 포트 열기
                ser = serial.serial_for_url(PORT, baudrate=115200, timeout=1)
                PORT = find_mbed_port(sudoPassword)
                isconnect = True
            except:
                count_connection += 1
                self.print_info(f"try to connection... {count_connection}")
                time.sleep(1.0)
                continue

        return ser

    # ROS2 초기화
    def init_ROS(self):
        # TF 송수신용 객체 정의(리소스 할당)
        # TF: 프레임의 자세, 위치 등을 정의하기 위한 메세지 컨테이너 포맷
        self.br = tf2_ros.TransformBroadcaster(self)
        self.br_static = tf2_ros.StaticTransformBroadcaster(self)
        self.ref_frame = "odom"

        # 현재 로봇 스테이터스를 송신하기 위한 객체 정의(리소스 할당)
        self.pub_t265 = self.create_publisher(Twist, '/cur_vel_t265', 10)
        self.pub_encoder = self.create_publisher(Twist, '/cur_vel_encoder', 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_mode = self.create_publisher(String, "/platform_mode", 10)
        self.pub_cmd_LPF = self.create_publisher(Twist, "/cmd_LPF", 10)
        self.pub_cmd_cleaning = self.create_publisher(Twist, "/cmd_cleaning", 10)
        self.pub_e_vx = self.create_publisher(Float32, '/error_vx', 10)
        self.pub_e_vy = self.create_publisher(Float32, '/error_vy', 10)
        self.pub_e_dyaw = self.create_publisher(Float32, '/error_dyaw', 10)

        self.buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.buffer, self)

        # QoS(Quality of Service) 프로필 정의
        # QoS: 데이터 송수신에서 데이터 송수신 품질을 정의함.
        #      데이터 신뢰도, 생애 주기, 버퍼 크기 등을 설정할 수 있음.
        #      Pub, Sub에서 해당 프로필이 일치하지 않는 경우 송수신이 되지 않는 경우가 생길 수 있음.
        custom_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            avoid_ros_namespace_conventions=False
        )

        # 자율 주행 패키지에서 계산한 목표 명령을 받기 위한 객체 정의(리소스 할당)
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.callback_cmd, 10)
        self.sub_cmd_gui = self.create_subscription(String, "/cmd_controller", self.callback_gui, 10)
        self.sub_target_yaw = self.create_subscription(Float32, "/yaw_for_cleaning", self.callback_target_yaw_cleaning, 10)
        self.sub_controller = self.create_subscription(String, "/selected_controller", self.callback_controller, custom_profile)
        self.sub_cmd_vision = self.create_subscription(String, "/cmd_vision", self.callback_vision, 10)

        self.cmd_vel = Twist()
        self.cmd_vel_LPF = Twist()
        self.cmd_vel_cleaning = Twist()
        self.isCleaning = False

        # dt 계산에 쓰일 현지 시간, 이전 시간에 대한 변수 할당
        # dt: 한 주기동안 소요되는 시간. 일반적으로 Sampling Time이라고 지칭함. LPF, 속도 계산 등에 쓰임
        self.cur_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()

    # 플랫폼 파라미터 초기화
    def init_platform(self):
        # 플랫폼 모드와 관련된 변수 초기화
        self.isteleop = False
        self.isConnected = False
        self.mode = cmd_mode.NULL
        self.set_cmd_mode(self.get_parameter('mode').value)

        # 로봇의 현재 위치 변수 초기화
        self.x_t265 = 0.0
        self.y_t265 = 0.0
        self.x_encoder = 0.0
        self.y_encoder = 0.0
        self.x_fusion = 0.0
        self.y_fusion = 0.0

        if(self.isRealPlatform):
            # real platform
            # 바퀴 / 롤러 반지름
            # 롤러: 바퀴 내부에 사선으로 배치된 작은 바퀴들.
            self.R = 0.0635
            self.r = 0.014
            self.sx = 0.29
            self.sy = 0.26

            # 라이다 위치 / 바라보는 자세 (Yaw)
            self.x_init_1 =  0.412
            self.y_init_1 =  0.272425
            self.yaw_init_1 = -180.8

            self.x_init_2 = -0.447
            self.y_init_2 = -0.302
            self.yaw_init_2 = 0
        else:
            # mini platform
            self.R = 0.0300
            self.r = 0.0057
            self.sx = 0.194
            self.sy = 0.1125

            self.x_init_1 =  0.140
            self.y_init_1 =  0.155
            self.yaw_init_1 = -119.50

            self.x_init_2 = -0.165
            self.y_init_2 = -0.160
            self.yaw_init_2 = 60.10

        # 로봇 자세 변수 초기화
        self.yaw_init_encoder = None
        self.yaw_init_t265 = None

        # 프로토콜 수신을 위한 버퍼 초기화
        self.str_buff = str()

    # 명령 모드 세팅
    def set_cmd_mode(self, mode: str):
        str_mode = mode
        if(str_mode == "NULL"):
            self.mode = cmd_mode.NULL
        elif(str_mode == "OPENLOOP"):
            self.mode = cmd_mode.OPENLOOP
        elif(str_mode == "FEEDBACK"):
            self.mode = cmd_mode.FEEDBACK
        elif(str_mode == "NAVIGATION"):
            self.mode = cmd_mode.NAVIGATION
        else:
            self.mode = cmd_mode.NULL
        self.print_info(f"Set cmd_mode to {str_mode}")

    # T265 초기화
    def init_realsense(self,json_path=get_package_share_directory('platform_control') + '/config/calibration_odometry.json'):
        # 리얼센스 디바이스 리스트 불러오기
        ctx = rs.context()
        devices = ctx.query_devices()
        devices_dic = {a.get_info(rs.camera_info.name):
                       a.get_info(rs.camera_info.serial_number)
                       for a in devices}

        # Intel RealSense T265 초기화 시도
        try:
            self.pipe_t265 = rs.pipeline()
            self.cfg_t265 = rs.config()

            # T265 활성화 및 자세 데이터 스트리밍 설정 불러오기
            self.cfg_t265.enable_device(devices_dic['Intel RealSense T265'])
            self.cfg_t265.enable_stream(rs.stream.pose)
            profile : rs.pipeline_profile = self.cfg_t265.resolve(self.pipe_t265)
            # T265 휠 오도메트리 기능 설정 불러오기
            self.t265_WheelOdom = self.load_t265_wheelodom_config(profile,json_path)
            self.x_robot_to_t265 = -0.36
            self.y_robot_to_t265 = 0.0

            self.print_info(f"x: {self.x_robot_to_t265} / y:{self.y_robot_to_t265}")
            # T265 활성화
            self.pipe_t265.start(self.cfg_t265)
            self.print_info("Wait for Initalization for T265...")
            time.sleep(2.0)
        # 예외처리
        except:
            self.print_info("Can't load Intel RealSense T265!")
            self.shutdown_node()

    # 피드백 관련 파라미터 초기화
    def init_feedback(self):
        P_gain_vx = 1.0
        I_gain_vx = 0
        D_gain_vx = 0

        P_gain_vy = 1.0
        I_gain_vy = 0
        D_gain_vy = 0

        P_gain_dyaw = 0.3
        I_gain_dyaw = 0
        D_gain_dyaw = 0

        P_gain_yaw = 1.0
        I_gain_yaw = 0.05
        D_gain_yaw = 2.0

        self.vx_feedback = PID_manager(P_gain_vx, I_gain_vx, D_gain_vx, cmd_threshold=0.05, I_threshold=0.02)
        self.vy_feedback = PID_manager(P_gain_vy, I_gain_vy, D_gain_vy, cmd_threshold=0.05, I_threshold=0.02)
        self.dyaw_feedback = PID_manager(P_gain_dyaw, I_gain_dyaw, D_gain_dyaw, cmd_threshold=0.05, I_threshold=0.02)

        self.yaw_feedback = PID_manager(P_gain_yaw, I_gain_yaw, D_gain_yaw, cmd_threshold=0.1, I_threshold=0.1)

    # 키보드 입력 관련 초기화
    def init_teleop(self):
        self.listener = keyboard.Listener(
            on_press=self.on_press_teleop, on_release=self.on_release_teleop)
        self.listener.start()

    # T265 wheel odometry 기능 초기화
    # wheel odometry: IMU + 엔코더로 위치 추정 함께 진행하는 경우 해당 정보를
    #                 T265 측으로 송신하면 위치 추정 성능이 개선됨.
    def load_t265_wheelodom_config(self, profile, json_path: str):
        dev = profile.get_device()
        tm2 = dev.as_tm2()

        pose_sensor = tm2.first_pose_sensor()
        wheel_odometer = pose_sensor.as_wheel_odometer()

        f = open(json_path)

        chars = []
        for line in f:
            for c in line:
                chars.append(ord(c))  # char to uint8

        wheel_odometer.load_wheel_odometery_config(chars)

        return wheel_odometer

    # ---------------------------------- main ---------------------------------- #
    def timer_callback(self):
        # 프로토콜 디코드
        try:
            dict_dr = self.protocol_decode(self.ser.read_all().decode())
        except Exception as e:
            self.print_info(f"decode_error: {e}")
            return

        # 디코드가 정상적으로 이루어졌는지 확인
        if  (dict_dr is None) or \
            not (("VX" in dict_dr) & ("VY" in dict_dr) & ("Y" in dict_dr)):
            return

        # 데드레코닝 관련 변수 초기화
        vx_dr = dict_dr["VX"]
        vy_dr = dict_dr["VY"]

        # IMU를 통한 Yaw 계산
        yaw_encoder = dict_dr['Y'] / 180.0 * math.pi

        # T265 초기화가 이루어졌는지 확인
        if (hasattr(self, "pipe_t265")) & (dict_dr.__len__() > 3):
            # T265 휠 오도메트리 기능 동작을 위해 하위 제어기에서 계산한 속도 데이터 송신
            self.send_t265_odom(vx_dr,vy_dr)
            # T265 주행기록계 데이터 수신
            vx_t265, vy_t265, x_t265, y_t265, yaw_t265 = self.get_pose_t265()

            # 데이터 유효성 확인
            if(vx_t265 is None):
                return

            # Sampling Time 계산
            dt = self.calculate_dt()

            # 각속도 계산
            if hasattr(self,"prev_yaw_encoder"):
                d_yaw_encoder = (yaw_encoder - self.prev_yaw_encoder) / dt
                self.prev_yaw_encoder = yaw_encoder

            if hasattr(self,"prev_yaw_t265"):
                d_yaw_t265 = (yaw_t265 - self.prev_yaw_t265) / dt
                self.prev_yaw_t265 = yaw_t265

            # T265 카메라 주행기록계 데이터를 카메라 기준으로 보정해주는 작업
            x_robot_t265, y_robot_t265, cur_vel_t265 = self.calculate_t265_to_robot_frame(x_t265, y_t265,
                                                                                          vx_t265, vy_t265,
                                                                                          d_yaw_t265, yaw_t265)

            # 데이터를 ROS2 메시지 형태로 변환하는 작업
            cur_vel_encoder = self.get_twist(vx_dr, vy_dr, d_yaw_encoder)

            # 앞에서 계산한 하위제어기, T265의 속도 데이터를 이용하여 플랫폼 위치를 추정하는 작업
            self.calculate_x_y_raw(cur_vel_t265, cur_vel_encoder, yaw_t265, yaw_t265, dt)

            # T265와 하위제어기에서 추정한 위치 데이터를 보정하는 작업
            self.sensor_fusion(x_robot_t265, y_robot_t265, self.x_encoder, self.y_encoder, yaw_t265)

            # 최종적으로 계산한 위치 데이터를 Publish하는 과정
            self.broadcast_pose(self.x_fusion, self.y_fusion, self.x_encoder, self.y_encoder, x_robot_t265,
                                y_robot_t265, yaw_t265, yaw_t265, cur_vel_encoder)

            # 플랫폼 모드를 Publish하는 과정
            self.broadcast_mode(self.mode)

            # 플랫폼이 정상 작동중인지 확인
            if not self.isConnected:
                self.print_info("Try to broadcasting...")
                self.isConnected = True

            # 플랫폼 모드에 따라 주행이 다르게 이루어짐
            if (self.mode == cmd_mode.FEEDBACK):
                self.command_vel(self.cmd_vel)
                self.pub_cmd_LPF.publish(self.cmd_vel)

            elif (self.mode == cmd_mode.NAVIGATION):
                if self.isCleaning:
                    cmd_vel_cleaning = self.calculate_cmd_cleaning(self.cmd_vel_LPF, dt)
                    self.command_vel(cmd_vel_cleaning)

                    self.pub_cmd_LPF.publish(self.cmd_vel_LPF)
                    self.pub_cmd_cleaning.publish(cmd_vel_cleaning)
                else:
                    cmd_vel_FB = self.calculate_feedback(self.cmd_vel_LPF,cur_vel_t265,dt)
                    # cmd_vel_FB = self.cmd_vel_LPF
                    self.command_vel(cmd_vel_FB)
                    self.pub_cmd_LPF.publish(cmd_vel_FB)
                    self.pub_e_vx.publish(Float32(data=cmd_vel_FB.linear.x - cur_vel_t265.linear.x))
                    self.pub_e_vy.publish(Float32(data=cmd_vel_FB.linear.y - cur_vel_t265.linear.y))
                    self.pub_e_dyaw.publish(Float32(data=cmd_vel_FB.angular.z - cur_vel_t265.angular.z))
                    # self.command_vel(self.cmd_vel_LPF)
                    # self.pub_cmd_LPF.publish(self.cmd_vel_LPF)
                    #
                    # self.pub_vx.publish(Float32(data=self.cmd_vel_LPF.linear.x))
                    # self.pub_vy.publish(Float32(data=self.cmd_vel_LPF.linear.y))

    # ---------------------------------- main ---------------------------------- #

    def sensor_fusion(self, x_robot_t265, y_robot_t265, x_encoder, y_encoder, yaw,
                      # cur_vel_encoder: Twist, cur_vel_t265: Twist
                      ):
        dx_robot_t265 = 0.0
        dy_robot_t265 = 0.0
        dx_encoder = 0.0
        dy_encoder = 0.0
        dyaw = 0.0

        if hasattr(self,"prev_x_robot_t265"):
            dx_robot_t265 = x_robot_t265 - self.prev_x_robot_t265
        if hasattr(self,"prev_y_robot_t265"):
            dy_robot_t265 = y_robot_t265 - self.prev_y_robot_t265
        if hasattr(self,"prev_x_encoder"):
            dx_encoder = x_encoder - self.prev_x_encoder
        if hasattr(self,"prev_y_encoder"):
            dy_encoder = y_encoder - self.prev_y_encoder
        if hasattr(self,"prev_yaw"):
            dyaw = yaw - self.prev_yaw

        # T265가 우세한 상황: 병진 운동, 외란 발생
        # 데드레코닝이 우세한 상황: 회전 운동, 정지

        # 로봇 상태 관측 방법
        # 정지 : vx, vy dyaw ~= 0

        # 회전 + 병진 : a / (a+b), b / (a+b)
        # 다만 보정 계수가 필요함.

        k1 = 1.5
        k2 = 4.0

        threshold_meter_teleport = 0.15

        if (dyaw != 0) or (dx_robot_t265 != 0) or (dx_encoder != 0):
            dv_t265 = math.sqrt(dx_robot_t265 * dx_robot_t265 + dy_robot_t265 * dy_robot_t265)
            dv_dr = math.sqrt(dx_encoder * dx_encoder + dy_encoder * dy_encoder)
            f1 = math.fabs(dyaw)
            f2 = dv_dr

            if dv_dr !=0:
                weight_dr = (k1 * f1) / (k1 * f1 + k2 * f2)
            else:
                weight_dr = 1.0

            weight_t265 = 1.0 - weight_dr

            # self.print_info(f"w_dr: {weight_dr:.3f} / w_t265: {weight_t265:.3f}")

            self.x_fusion += weight_t265 * dx_robot_t265 + weight_dr * dx_encoder
            self.y_fusion += weight_t265 * dy_robot_t265 + weight_dr * dy_encoder

            check_w_t265 = weight_t265 > 0.9
            check_teleport = (dv_t265 - dv_dr > threshold_meter_teleport)

            if check_w_t265 and check_teleport:
                self.print_info("Teleport Occured!")
                self.x_fusion = x_robot_t265
                self.y_fusion = y_robot_t265

        self.prev_x_robot_t265 = x_robot_t265
        self.prev_y_robot_t265 = y_robot_t265
        self.prev_x_encoder = x_encoder
        self.prev_y_encoder = y_encoder
        self.prev_yaw = yaw

    def timer_callback_test(self):
        if self.pipe_t265 is not None:
            vx_t265, vy_t265, x_t265, y_t265, yaw_rad_t265 = self.get_pose_t265()

    def protocol_decode(self,str_msg : str):
        self.str_buff += str_msg
        dict_str = dict()

        try:
            str_split = self.str_buff.split('\n')

            if str_split.__len__() == 0: return dict_str

            str_0 = ""

            for i in reversed(str_split):
                if ('<' in i) & ('>' in i) & ('VX:' in i) & (';Y:' in i):
                    str_0 = i
                    self.str_buff = ""
                    break

            if (str_0 != ""):
                str_1 = str_0.split('<')
                str_inner = str_1[str_1.__len__() - 1].split('>')[0]
                str_dict = str_inner.split(';')
                str_dict.remove('')
                dict_str = {a.split(':')[0]: float(a.split(':')[1])
                            for a in str_dict}

                if self.yaw_init_encoder is None:
                    self.yaw_init_encoder = dict_str['Y']
                    self.prev_yaw_encoder = 0
                    self.prev_time = self.get_clock().now()

                dict_str['Y'] = dict_str['Y'] - self.yaw_init_encoder

            return dict_str

        except Exception as er:
            self.print_info(f"Decode Error: {er}")
            self.str_buff = ""
            return None

    def get_pose_t265(self):
        try:
            frames = self.pipe_t265.wait_for_frames()
            pose = frames.get_pose_frame()

            if pose:
                data = pose.get_pose_data()
                tfv = [-data.velocity.z, -data.velocity.x, data.velocity.y]
                q = [data.rotation.z, data.rotation.x, -data.rotation.y, data.rotation.w]
                q = quaternion_multiply(q, [0, 0, -1, 0])
                vx, vy, vz = qv_mult(q, tfv)

                r, p, y = euler_from_quaternion(q[0], q[1], q[2], q[3])

                if self.yaw_init_t265 is None:
                    self.yaw_init_t265 = y
                    self.prev_yaw_t265 = 0

                yaw_t265 = y - self.yaw_init_t265
                if(self.isRealPlatform):
                    return vx, vy, data.translation.z, data.translation.x, -yaw_t265
                else:
                    return -vx, -vy, -data.translation.z, -data.translation.x, yaw_t265

        except RuntimeError as e:
            print(e)
            return None, None, None, None, None

    def get_twist(self, vx, vy, dyaw):
        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = dyaw
        return tw

    def send_t265_odom(self, vx, vy):
        if(hasattr(self,"t265_WheelOdom")):
            wo_sensor_id = 0  # indexed from 0, match to order in calibration file
            frame_num = 0  # not used
            v = rs.vector()
            v.z = vx  # m/s
            v.x = vy

            self.t265_WheelOdom.send_wheel_odometry(wo_sensor_id, frame_num, v)

    def calculate_t265_to_robot_frame(self, x_t265, y_t265, vx_t265, vy_t265, d_yaw_t265, yaw_t265):

        L = math.sqrt((self.x_robot_to_t265 * self.x_robot_to_t265) + (self.y_robot_to_t265 * self.y_robot_to_t265))

        x_robot = x_t265 + (L * math.cos(yaw_t265) + self.x_robot_to_t265)
        y_robot = y_t265 + (L * math.sin(yaw_t265) + self.y_robot_to_t265)

        vx_robot = vx_t265
        vy_robot = vy_t265 + L * d_yaw_t265

        cur_vel = self.get_twist(vx_robot, vy_robot, d_yaw_t265)

        return x_robot, y_robot, cur_vel

    def calculate_dt(self):
        self.cur_time = self.get_clock().now()
        dt = (self.cur_time - self.prev_time).nanoseconds / 1000000000.0
        self.prev_time = self.get_clock().now()
        return dt

    def calculate_x_y_raw(self, cur_vel_t265: Twist, cur_vel_encoder: Twist, yaw_t265, yaw_encoder, dt):

        vx_t265 = cur_vel_t265.linear.x
        vy_t265 = cur_vel_t265.linear.y

        vx_encoder = cur_vel_encoder.linear.x
        vy_encoder = cur_vel_encoder.linear.y

        self.x_encoder += vx_encoder * math.cos(yaw_encoder) * dt
        self.x_encoder += vy_encoder * math.cos(yaw_encoder + math.pi / 2) * dt
        self.y_encoder += vx_encoder * math.sin(yaw_encoder) * dt
        self.y_encoder += vy_encoder * math.sin(yaw_encoder + math.pi / 2) * dt

        self.x_t265 += vx_t265 * math.cos(yaw_t265) * dt
        self.x_t265 += vy_t265 * math.cos(yaw_t265 + math.pi / 2) * dt
        self.y_t265 += vx_t265 * math.sin(yaw_t265) * dt
        self.y_t265 += vy_t265 * math.sin(yaw_t265 + math.pi / 2) * dt

        self.pub_t265.publish(cur_vel_t265)
        self.pub_encoder.publish(cur_vel_encoder)

        # print('encoder : {0:.3f}  {1:.3f}\nt265 : {2:.3f}  {3:.3f}'.format(
        #       vx_encoder, vy_encoder, vx_t265, vy_t265))

    def calculate_cmd_rpm(self, cmd_vel : Twist):
        RADPS_TO_RPM = 9.5493

        R = self.R      # 0.03
        r = self.r      # 0.0057
        sx = self.sx    # 0.194
        sy = self.sy    # 0.1125
        vx_cmd = cmd_vel.linear.x
        vy_cmd = cmd_vel.linear.y
        d_yaw_cmd = cmd_vel.angular.z

        pi_1 = (1 / (R + r)) * (vx_cmd - vy_cmd - d_yaw_cmd * (sx + sy))
        pi_2 = (1 / (R + r)) * (vx_cmd + vy_cmd + d_yaw_cmd * (sx + sy))
        pi_3 = (1 / (R + r)) * (vx_cmd + vy_cmd - d_yaw_cmd * (sx + sy))
        pi_4 = (1 / (R + r)) * (vx_cmd - vy_cmd + d_yaw_cmd * (sx + sy))

        rpm_1 = pi_1 * RADPS_TO_RPM
        rpm_2 = pi_2 * RADPS_TO_RPM
        rpm_3 = pi_3 * RADPS_TO_RPM
        rpm_4 = pi_4 * RADPS_TO_RPM

        return rpm_1, rpm_2, rpm_3, rpm_4

    def broadcast_pose(self, x, y, x_encoder, y_encoder, x_t265, y_t265, yaw_t265, yaw_encoder, cur_vel):
        now_stamp = self.get_clock().now().to_msg()

        t0 = TransformStamped()
        t0.header.stamp = now_stamp
        t0.header.frame_id = self.ref_frame
        t0.child_frame_id = "robot_base"
        t0.transform.translation.x = x
        t0.transform.translation.y = y
        t0.transform.translation.z = 0.0

        q0 = quaternion_from_euler(0, 0, yaw_t265)

        t0.transform.rotation.w = q0[0]
        t0.transform.rotation.x = q0[1]
        t0.transform.rotation.y = q0[2]
        t0.transform.rotation.z = q0[3]

        t1 = TransformStamped()
        t1.header.stamp = now_stamp
        # t1.header.frame_id = self.ref_frame
        t1.header.frame_id = "map"
        t1.child_frame_id = "robot_base_encoder"
        t1.transform.translation.x = x_encoder
        t1.transform.translation.y = y_encoder
        t1.transform.translation.z = 0.0

        q1 = quaternion_from_euler(0, 0, yaw_encoder)

        t1.transform.rotation.w = q1[0]
        t1.transform.rotation.x = q1[1]
        t1.transform.rotation.y = q1[2]
        t1.transform.rotation.z = q1[3]

        t2 = TransformStamped()
        t2.header.stamp = now_stamp
        # t2.header.frame_id = self.ref_frame
        t2.header.frame_id = "map"
        t2.child_frame_id = "robot_base_t265"
        t2.transform.translation.x = x_t265
        t2.transform.translation.y = y_t265
        t2.transform.translation.z = 0.0

        q2 = quaternion_from_euler(0, 0, yaw_t265)

        t2.transform.rotation.w = q2[0]
        t2.transform.rotation.x = q2[1]
        t2.transform.rotation.y = q2[2]
        t2.transform.rotation.z = q2[3]

        t3 = TransformStamped()
        t3.header.stamp = now_stamp
        t3.header.frame_id = "map"
        t3.child_frame_id = "robot_base_no_amcl"
        t3.transform.translation.x = x
        t3.transform.translation.y = y
        t3.transform.translation.z = 0.0

        q3 = quaternion_from_euler(0, 0, yaw_t265)

        t3.transform.rotation.w = q3[0]
        t3.transform.rotation.x = q3[1]
        t3.transform.rotation.y = q3[2]
        t3.transform.rotation.z = q3[3]

        self.broadcast_tf2_static()

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "robot_base"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = q1[0]
        odom.pose.pose.orientation.x = q1[1]
        odom.pose.pose.orientation.y = q1[2]
        odom.pose.pose.orientation.z = q1[3]

        odom.twist.twist = cur_vel

        self.pub_odom.publish(odom)
        self.br.sendTransform(t0)
        self.br.sendTransform(t1)
        self.br.sendTransform(t2)
        self.br.sendTransform(t3)

    def broadcast_mode(self, mode: cmd_mode):
        str_mode = String()

        if(mode is cmd_mode.NULL):
            str_mode.data = "NULL"
        elif(mode is cmd_mode.OPENLOOP):
            str_mode.data = "OPENLOOP"
        elif(mode is cmd_mode.FEEDBACK):
            str_mode.data = "FEEDBACK"
        elif(mode is cmd_mode.NAVIGATION):
            str_mode.data = "NAVIGATION"

        self.pub_mode.publish(str_mode)

    def calculate_cmd_cleaning(self, cmd_vel: Twist, dt: float):
        if(self.isCleaning and hasattr(self,'target_yaw')):
            tf_now = self.get_robot_base()

            _, _, cur_yaw = euler_from_quaternion(tf_now.transform.rotation.x,
                                                  tf_now.transform.rotation.y,
                                                  tf_now.transform.rotation.z,
                                                  tf_now.transform.rotation.w)

            error = self.target_yaw - cur_yaw

            # if error > math.pi:
            #     error -= 2 * math.pi
            cmd_vel_cleaning = Twist()
            cmd_angular_vel = self.yaw_feedback.calculate_cmd(error,dt)

            cmd_vel_cleaning.linear.x = cmd_vel.linear.x
            cmd_vel_cleaning.linear.y = cmd_vel.linear.y
            cmd_vel_cleaning.angular.z = cmd_vel.angular.z + cmd_angular_vel
            # cmd_vel_cleaning.angular.z = cmd_angular_vel

            self.print_info(f"e: {error} / cmd: {cmd_angular_vel}")

            return cmd_vel_cleaning
        else:
            return Twist()

    def calculate_feedback(self, cmd_vel : Twist, cur_vel : Twist, dt):
        vx_error = cmd_vel.linear.x - cur_vel.linear.x
        vy_error = cmd_vel.linear.y - cur_vel.linear.y
        dyaw_error = cmd_vel.angular.z - cur_vel.angular.z

        cmd_vx_FB = self.vx_feedback.calculate_cmd(vx_error, dt)
        cmd_vy_FB = self.vy_feedback.calculate_cmd(vy_error, dt)
        cmd_dyaw_FB = self.dyaw_feedback.calculate_cmd(dyaw_error, dt)

        cmd_tw = Twist()
        cmd_tw.linear.x = cmd_vel.linear.x + cmd_vx_FB
        cmd_tw.linear.y = cmd_vel.linear.y + cmd_vy_FB
        cmd_tw.angular.z = cmd_vel.angular.z + cmd_dyaw_FB

        if cmd_vel.linear.x == 0.0:
            cmd_tw.linear.x = 0.0
        if cmd_vel.linear.y == 0.0:
            cmd_tw.linear.y = 0.0
        if cmd_vel.angular.z == 0.0:
            cmd_tw.angular.z = 0.0

        # self.pub_error.publish(cmd_tw)
        # print(f"vx: {vx_error:.3f} / vy: {vy_error:.3f} / dyaw: {dyaw_error:.3f}")

        return cmd_tw

    def command_vel(self, cmd_vel: Twist, k=1.25):
        rpm_LF, rpm_RF, rpm_LB, rpm_RB = self.calculate_cmd_rpm(cmd_vel)

        str_cmd = get_str_cmdFeedback(rpm_LF * k, rpm_RF * k, rpm_LB * k, rpm_RB * k)
        self.ser.write(str_cmd.encode())

    def broadcast_tf2_static(self):
        now_stamp = self.get_clock().now().to_msg()

        t1 = TransformStamped()
        t1.header.stamp = now_stamp
        t1.header.frame_id = "robot_base"
        t1.child_frame_id = "lidar_1_raw"

        t1.transform.translation.x = self.x_init_1
        t1.transform.translation.y = self.y_init_1
        t1.transform.translation.z = 0.0

        q1 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_init_1))

        t1.transform.rotation.w = q1[0]
        t1.transform.rotation.x = q1[1]
        t1.transform.rotation.y = q1[2]
        t1.transform.rotation.z = q1[3]

        t2 = TransformStamped()
        t2.header.stamp = now_stamp
        t2.header.frame_id = "robot_base"
        t2.child_frame_id = "lidar_2_raw"

        t2.transform.translation.x = self.x_init_2
        t2.transform.translation.y = self.y_init_2
        t2.transform.translation.z = 0.0

        q2 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_init_2))

        t2.transform.rotation.w = q2[0]
        t2.transform.rotation.x = q2[1]
        t2.transform.rotation.y = q2[2]
        t2.transform.rotation.z = q2[3]

        t3 = TransformStamped()
        t3.header.stamp = now_stamp
        t3.header.frame_id = "robot_base"
        t3.child_frame_id = "lidar_1"

        t3.transform.translation.x = self.x_init_1
        t3.transform.translation.y = self.y_init_1
        t3.transform.translation.z = 0.0

        q3 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_init_1+180.0))

        t3.transform.rotation.w = q3[0]
        t3.transform.rotation.x = q3[1]
        t3.transform.rotation.y = q3[2]
        t3.transform.rotation.z = q3[3]

        t4 = TransformStamped()
        t4.header.stamp = now_stamp
        t4.header.frame_id = "robot_base"
        t4.child_frame_id = "lidar_2"

        t4.transform.translation.x = self.x_init_2
        t4.transform.translation.y = self.y_init_2
        t4.transform.translation.z = 0.0

        q4 = quaternion_from_euler(0, 0, deg_to_rad(self.yaw_init_2+180.0))

        t4.transform.rotation.w = q4[0]
        t4.transform.rotation.x = q4[1]
        t4.transform.rotation.y = q4[2]
        t4.transform.rotation.z = q4[3]

        t5 = TransformStamped()
        t5.header.stamp = now_stamp
        t5.header.frame_id = "robot_base"
        t5.child_frame_id = "base_footprint"

        t5.transform.translation.x = 0.0
        t5.transform.translation.y = 0.0
        t5.transform.translation.z = 0.0

        t5.transform.rotation.w = 1.0
        t5.transform.rotation.x = 0.0
        t5.transform.rotation.y = 0.0
        t5.transform.rotation.z = 0.0

        t6 = TransformStamped()
        t6.header.stamp = now_stamp
        t6.header.frame_id = "robot_base"
        t6.child_frame_id = "realsense_d455"

        t6.transform.translation.x = 0.0
        t6.transform.translation.y = 0.0
        t6.transform.translation.z = 0.925

        t6.transform.rotation.w = 1.0
        t6.transform.rotation.x = 0.0
        t6.transform.rotation.y = 0.0
        t6.transform.rotation.z = 0.0

        self.br.sendTransform(t1)
        self.br.sendTransform(t2)
        self.br.sendTransform(t3)
        self.br.sendTransform(t4)
        self.br.sendTransform(t5)
        self.br.sendTransform(t6)

    def on_press_teleop(self, key):
        if (self.mode == cmd_mode.OPENLOOP):
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
                print(str_cmd)
                self.ser.write(str_cmd.encode())
            except:
                return

        if (self.mode == cmd_mode.FEEDBACK):
            try:
                if (key.char == "w"):
                    self.cmd_vel.linear.x += 0.02
                if (key.char == "s"):
                    self.cmd_vel.linear.x -= 0.02
                if (key.char == "a"):
                    self.cmd_vel.linear.y += 0.02
                if (key.char == "d"):
                    self.cmd_vel.linear.y -= 0.02
                if (key.char == "q"):
                    self.cmd_vel.angular.z += 0.04
                if (key.char == "e"):
                    self.cmd_vel.angular.z -= 0.04

            except AttributeError:
                if (key.name == "space"):
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.linear.y = 0.0

    def on_release_teleop(self, key):
        try:
            if (key.name == "f2"):
                if (self.mode != cmd_mode.OPENLOOP):
                    self.print_info("Teleop mode ON!")
                    self.mode = cmd_mode.OPENLOOP
                else:
                    self.print_info("Teleop mode OFF!")
                    self.mode = cmd_mode.NULL
                    self.ser.write(get_str_command(platform_mode.Stop, 0).encode())

            if (key.name == "f3"):
                if (self.mode != cmd_mode.FEEDBACK):
                    self.print_info("Feedback mode ON!")
                    self.mode = cmd_mode.FEEDBACK
                else:
                    self.print_info("Feedback mode OFF!")
                    self.mode = cmd_mode.NULL
                    self.ser.write(get_str_command(platform_mode.Stop, 0).encode())

            if (key.name == "f4"):
                if (self.mode != cmd_mode.NAVIGATION):
                    self.print_info("Navigation mode ON!")
                    self.mode = cmd_mode.NAVIGATION
                else:
                    self.print_info("Navigation mode OFF!")
                    self.mode = cmd_mode.NULL
                    self.ser.write(get_str_command(platform_mode.Stop, 0).encode())


            if (key.name == "f10"):
                self.ser.write(get_str_command(platform_mode.Stop, 0).encode())
                self.destroy_node()
                os._exit(os.EX_OK)
        except:
            return

    def shutdown_node(self):
        self.destroy_node()
        os._exit(os.EX_OK)

    def print_info(self, str_info: str):
        self.get_logger().info(str_info)

    def get_robot_base(self, target="robot_base"):
        try:
            tf = self.buffer.lookup_transform("map", target, rclpy.time.Time())  # Blocking
            return tf
        except Exception as e:
            self.info(f"error: {e}")
            return None

    def callback_cmd(self, msg : Twist):
        if(self.mode == cmd_mode.NAVIGATION):
            self.cmd_vel = msg

            if((self.cmd_vel_LPF.linear.x == 0)):
                self.cmd_vel_LPF = msg
            else:
                self.cmd_vel_LPF.linear.x = 0.9 * self.cmd_vel_LPF.linear.x + 0.1 * self.cmd_vel.linear.x
                self.cmd_vel_LPF.linear.y = 0.9 * self.cmd_vel_LPF.linear.y + 0.1 * self.cmd_vel.linear.y
                self.cmd_vel_LPF.angular.z = 0.9 * self.cmd_vel_LPF.angular.z + 0.1 * self.cmd_vel.angular.z

                if (self.cmd_vel.linear.x == 0.0): self.cmd_vel_LPF.linear.x = 0.0
                if (self.cmd_vel.linear.y == 0.0): self.cmd_vel_LPF.linear.y = 0.0
                if (self.cmd_vel.angular.z == 0.0): self.cmd_vel_LPF.angular.z = 0.0

    def callback_gui(self, msg : String):
        if(self.ser.writable()):
            self.ser.write(msg.data.encode())
            self.ser.write(msg.data.encode())
            self.ser.write(msg.data.encode())
            self.print_info(f"send: {msg.data.encode()}")
        else:
            self.print_info(f"Can't send {msg.data.encode()}'")

    def callback_vision(self, msg : String):
        print(msg.data)

    def callback_controller(self, msg : String):
        str_msg = msg.data
        if(str_msg == "Cleaning"):
            self.isCleaning = True
        else:
            self.isCleaning = False

        self.print_info(f"isCleaning: {self.isCleaning}")

    def callback_target_yaw_cleaning(self, msg: Float32):
        self.target_yaw = msg.data

def main(args=None):
    rclpy.init(args=args)
    # 포트 설정
    PORT_mbed = find_mbed_port(sudoPassword)
    node = PlatformControlNode(PORT_mbed)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
