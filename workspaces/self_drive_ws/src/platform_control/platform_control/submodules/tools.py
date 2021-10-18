import enum
import numpy as np
import math
from geometry_msgs.msg import Quaternion
import subprocess
from ament_index_python.packages import get_package_share_directory
import fcntl
import os
import re
import sys

def quaternion_multiply(quaternion1, quaternion2):
    """Return multiplication of two quaternions.
    q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    numpy.allclose(q, [-44, -14, 48, 28])
    True
    """
    x1, y1, z1, w1 = quaternion1
    x2, y2, z2, w2 = quaternion2

    """
    return Quaternion(q1.w()*q2.x() + q1.x()*q2.w() + q1.y()*q2.z() - q1.z()*q2.y(),
                      q1.w()*q2.y() + q1.y()*q2.w() + q1.z()*q2.x() - q1.x()*q2.z(),
                      q1.w()*q2.z() + q1.z()*q2.w() + q1.x()*q2.y() - q1.y()*q2.x(),
                      q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z());
    """

    return np.array((
         w1*x2 + x1*w2 + y1*z2 - z1*y2,
         w1*y2 + y1*w2 + z1*x2 - x1*z2,
         w1*z2 + z1*w2 + x1*y2 - y1*x2,
         w1*w2 - x1*x2 - y1*y2 - z1*z2), dtype=np.float64)

def quaternion_multiply_q_v(q, v):
    """Return multiplication of two quaternions.
    q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    numpy.allclose(q, [-44, -14, 48, 28])
    True
    """
    qx, qy, qz, qw = q
    wx, wy, wz = v

    """
    return Quaternion( q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
					   q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
					   q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
		              -q.x() * w.x() - q.y() * w.y() - q.z() * w.z());
    """

    return np.array((
        qw*wx + qy*wz - qz*wy,
        qw*wy + qz*wx - qx*wz,
        qw*wz + qx*wy - qy*wx,
       -qx*wx - qy*wy - qz*wz), dtype=np.float64)

def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. eucledian norm, along axis.
    v0 = numpy.random.random(3)
    v1 = unit_vector(v0)
    numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
    True
    v0 = numpy.random.rand(5, 4, 3)
    v1 = unit_vector(v0, axis=-1)
    v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
    numpy.allclose(v1, v2)
    True
    v1 = unit_vector(v0, axis=1)
    v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
    numpy.allclose(v1, v2)
    True
    v1 = numpy.empty((5, 4, 3), dtype=numpy.float64)
    unit_vector(v0, axis=1, out=v1)
    numpy.allclose(v1, v2)
    True
    list(unit_vector([]))
    []
    list(unit_vector([1.0]))
    [1.0]
    """
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data*data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data

def quaternion_conjugate(quaternion):
    """Return conjugate of quaternion.
    q0 = random_quaternion()
    q1 = quaternion_conjugate(q0)
    q1[3] == q0[3] and all(q1[:3] == -q0[:3])
    True
    """
    return np.array((-quaternion[0], -quaternion[1],
                     -quaternion[2],  quaternion[3]), dtype=np.float64)

def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[:3]

    # q = quaternion_multiply_q_v(q1,v1)
    # q = quaternion_multiply(q, quaternion_conjugate(q1))
    # return [q[0], q[1], q[2]]

    # Quaternion q = q1 * v;
    # q *= q1.inverse();
    # return Vector3(q.getX(),q.getY(),q.getZ());

class platform_mode(enum.Enum):
    Front = "F"
    Back = "B"
    Stop = "S"
    LFront = "LF"
    RFront = "RF"
    Left = "L"
    Right = "R"
    LBack = "LB"
    RBack = "RB"
    TLeft = "TL"
    TRight = "TR"
    Feedback = "CMD"

class cmd_mode(enum.Enum):
    NULL = -1
    OPENLOOP = 0
    FEEDBACK = 1
    NAVIGATION = 2

def get_str_command(mode_: platform_mode, velo: float) -> object:
    return "<{0}:{1};>".format(mode_.value, velo)

def get_str_cmdFeedback(LF: float, RF: float, LB: float, RB: float) -> object:
    LF_new = limit(100, -100, LF)
    RF_new = limit(100, -100, RF)
    LB_new = limit(100, -100, LB)
    RB_new = limit(100, -100, RB)

    if ((LF == 0.0) & (RF == 0.0) & (LB == 0.0) & (RB == 0.0)):
        return get_str_command(platform_mode.Stop, 0)
    else:
        return f"<CMD:{LF_new:.3f},{RF_new:.3f},{LB_new:.3f},{RB_new:.3f};>"

def limit(max, min, val):
    val_new = val
    if (val > max):
        val_new = max
    elif (val < min):
        val_new = min
    return val_new

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

def set_bt_port(sudoPassword):
    command = 'rfcomm bind 20 98:D3:41:FD:59:D8'
    # p = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    output = subprocess.check_output('echo %s|sudo -S %s' % (sudoPassword, command), shell=True)
    return '/dev/rfcomm20'

def find_mbed_port(sudoPassword):
    bash_name = 'findTTY.bash'
    str_dir = get_package_share_directory('platform_control') + '/bash/' + bash_name

    command = 'bash {0}'.format(str_dir)

    output = subprocess.check_output('echo %s|sudo -S %s' % (sudoPassword, command), shell=True)
    print('\n')
    str_split = output.decode().split('\n')
    str_split.remove('')
    try:
        return [s for s in str_split if 'STMicroelectronics_STM32_STLink' in s][0].split(' - ')[0]
    except:
        return None

def deg_to_rad(deg : float):
    if deg == 0.0:
        return 0.0
    else:
        return deg * math.pi / 180.0

def reset_realsense_usb(id_realsense = "03e7:2150"):
    lsusb_cmd = f'lsusb | grep {id_realsense}'
    USBDEVFS_RESET = 21780
    print('Executing command: `{}`'.format(lsusb_cmd))

    lsusb_out = subprocess.check_output(lsusb_cmd, shell=True)
    print('Subprocess: ', lsusb_out.strip())

    parts = re.search(r'Bus (?P<bus>\d+) Device (?P<dev>\d+): ID [:\d\w]+ (?P<desc>.*)$', str(lsusb_out))
    bus = parts.group('bus')
    dev = parts.group('dev')
    desc = parts.group('desc').strip()
    print('Found device {} on bus {} for "{}"'.format(bus, dev, desc))

    f = open('/dev/bus/usb/{}/{}'.format(bus, dev), 'w', os.O_WRONLY)
    fcntl.ioctl(f, USBDEVFS_RESET, 0)

class PID_manager:
    def __init__(self, P, I, D, I_threshold=0.01, cmd_threshold=0.02):
        self.P_gain = P
        self.I_gain = I
        self.D_gain = D

        self.I_val = 0.0

        self.cmd_val = 0.0
        self.prev_e = 0.0

        self.I_threshold = I_threshold
        self.cmd_threshold = cmd_threshold

    def calculate_cmd(self, e, dt):
        p_val = self.P_gain * e
        self.I_val += self.I_gain * e

        if (self.I_val >= self.I_threshold):
            self.I_val = self.I_threshold
        elif (self.I_val <= -self.I_threshold):
            self.I_val = -self.I_threshold

        d_val = self.D_gain *((e - self.prev_e) / dt)
        self.prev_e = e

        cmd_value = (p_val + self.I_val + d_val)

        if (cmd_value >= self.cmd_threshold):
            cmd_value = self.cmd_threshold
        elif (cmd_value <= -self.cmd_threshold):
            cmd_value = -self.cmd_threshold

        return cmd_value
