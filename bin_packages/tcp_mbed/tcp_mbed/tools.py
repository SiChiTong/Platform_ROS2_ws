import enum
import numpy as np
import math
from geometry_msgs.msg import Quaternion
import subprocess
from ament_index_python.packages import get_package_share_directory

# sudoPassword = 'as449856'

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

def get_str_command(mode_: platform_mode, velo : float):
    return "<{0}:{1};>".format(mode_.value, velo)

def euler_from_quaternion(quaternion: Quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

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
    str_dir = get_package_share_directory('tcp_mbed') + '/bash/' + bash_name

    command = 'bash {0}'.format(str_dir)

    output = subprocess.check_output('echo %s|sudo -S %s' % (sudoPassword, command), shell=True)
    str_split = output.decode().split('\n')
    str_split.remove('')
    try:
        return [s for s in str_split if 'STMicroelectronics_STM32_STLink' in s][0].split(' - ')[0]
    except:
        return None

def deg_to_rad(deg : float):
    return deg * math.pi / 180.0