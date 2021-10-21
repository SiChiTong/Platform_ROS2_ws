#!/usr/bin/env python3
import math
import os

import std_msgs.msg
import sys
from ament_index_python.packages import get_package_share_directory
import open3d as o3d

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = BASE_DIR
SHARE_DIR = get_package_share_directory('detection_2d')

sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
sys.path.append(os.path.join(ROOT_DIR, 'models'))

import time

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np

from models.experimental import attempt_load
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized

import rclpy
from rclpy.node import Node
from pcd_manager.msg import RGBDImage, SerializedArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from pydoc import locate
from datetime import datetime

from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg

def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)

def img_preprocess(img0, imgsz, half, stride):
    img = [letterbox(x, imgsz, stride=stride)[0] for x in img0]
    # Stack
    img = np.stack(img, 0)
    # Convert
    img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    return img

def view(img):
    # data, t2, t1, img = detected_data.get()
    # cv2.putText(img, "FPS : %0.2f" % f, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
    cv2.imshow('VIEW', img)
    cv2.waitKey(1)

def detect(device, model, img_raw: np.ndarray, names, colors,
           imgsz=[448, 640], conf_thres=0.01, iou_thres=0.25, classes=None, agnostic_nms=False, isVisualized = True, verbose = False, Rsample = True):

    img_raw = np.expand_dims(img_raw, axis=0)

    half = device.type != 'cpu'  # half precision only supported on CUDA
    if half:
        model.half()  # to FP16

    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once
    t0 = time.time()

    img0 = img_raw
    img = img_preprocess(img0, imgsz, half, stride)

    # Inference
    t1 = time_synchronized()
    pred = model(img)[0]

    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
    t2 = time_synchronized()

    # Visualization
    det = pred[0]
    im0 = img0[0].copy()
    list_result = []

    if len(det):
        data1 = "Object detected! "
        data2 = "object classes : "
        # Rescale boxes from img_size to im0 size
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

        # Write results
        for *xyxy, conf, cls in reversed(det):
            label = f'{names[int(cls)]}:{conf:.2f}'
            plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=2)
            cl = names[int(cls)]
            center = (int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2))
            data2 = data2 + "[" + cl + " at" + str(center) + "] "
            c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
            list_result.append([names[int(cls)], c1, c2, float(conf)])

        send_data = data1 + data2
    else:
        send_data = "Not detected"

    # Print time (inference + NMS)
    if verbose:
        print(f'Done. ({t2 - t1:.3f}s)')
        print(send_data)

    if isVisualized: view(im0)

    return list_result

def loop_cam(cam_num: int):
    cam = cv2.VideoCapture(cam_num)

    while True:
        ret, img = cam.read()
        if ret:
            with torch.no_grad(): detect(device, model, img, names, colors,
                                         conf_thres=0.3, iou_thres=0.3)

class o3d_manager:
    def __init__(self, isVisualize=True):
        self.vis = o3d.visualization.Visualizer()
        self.pointcloud = o3d.geometry.PointCloud()
        self.isinit = False
        self.isVisualize = isVisualize
        self.bbox_prev = []

    def add_pcd_to_vis(self):
        if((not self.isinit) and (self.isVisualize)):
            self.vis.create_window("Test")
            self.isinit = self.vis.add_geometry(self.pointcloud)

    def pcd_show(self, pcd, bbox=[]):
        self.copy_pcd(pcd)
        self.add_pcd_to_vis()
        self.draw_bbox(bbox)

        self.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()

    def copy_pcd(self, pcd):
        self.pointcloud.points = pcd.points
        self.pointcloud.colors = pcd.colors

    def draw_bbox(self, bbox: list):
        for a in self.bbox_prev:
            self.vis.remove_geometry(a, reset_bounding_box=False)
        self.bbox_prev.clear()
        for a in bbox:
            self.vis.add_geometry(a, reset_bounding_box=False)
            self.bbox_prev.append(a)

    def update_geometry(self):
        self.vis.update_geometry(self.pointcloud)
        # for a in self.bbox_prev:
        #     self.vis.update_geometry(a)

class rate_mine():
    def __init__(self, rate):
        self.rate = rate
        self.dt0 = datetime.now()

    def sleep(self):
        process_time = (datetime.now() - self.dt0).total_seconds()
        period = 1.0 / self.rate
        if (process_time <= period):
            time.sleep(period - process_time)
        self.dt0 = datetime.now()

class detection_node(Node):
    def __init__(self, model, device):
        super().__init__("yolo")

        self.declare_parameter("show_running_time", value=True)
        self.show_FPS = self.get_parameter('show_running_time').value
        self.declare_parameter("rate", value=10.0)

        self.rate_pub = rate_mine(self.get_parameter('rate').value)
        self.model = model

        self.names = model.module.names if hasattr(model, 'module') else model.names  # get class names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]
        self.vis_o3d = o3d_manager()

        self.device = device

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.sub_rgbd = self.create_subscription(RGBDImage, '/pcd_realsense', self.callback_rgbd, qos_profile)
        self.pub_check = self.create_publisher(std_msgs.msg.String,'/check',1)

        self.print_info("Success for Initialization!")

    def callback_rgbd(self, msg: RGBDImage):
        dt0 = datetime.now()
        img: np.ndarray = self.unpack(msg.color)
        pcd: np.ndarray = self.unpack(msg.depth_to_xyz)

        pcd_total = o3d.geometry.PointCloud()
        bbox_total = []

        with torch.no_grad():

            # result : [label, point_Left_Up, point_Right_Down, confidence]

            result = detect(self.device, self.model, img, self.names, self.colors,
                            imgsz=[512, 640], conf_thres=0.2, iou_thres=0.3, isVisualized=True)

            pcd_img = np.concatenate((pcd, img), axis=2)

            for a in result:
                r1, r2 = a[1], a[2]
                pcd_img_bbox = pcd_img[r1[1]:r2[1], r1[0]:r2[0], ...]
                ab = ~np.isnan(pcd_img_bbox.reshape([-1, 6]))[:,0]
                aab = pcd_img_bbox.reshape([-1, 6])[ab]

                try:
                    ratio_nan = (np.size(ab) - np.count_nonzero(ab)) / np.size(ab)
                    if(ratio_nan>0.8): raise Exception("Too many Nan value..(skip)")

                    point_pcd_np = aab[:, 0:3]
                    color_pcd_np = (aab[:, 3:6] / 255.0)[..., ::-1]

                    pcd_bbox = o3d.geometry.PointCloud()

                    pcd_bbox.points = o3d.utility.Vector3dVector(point_pcd_np)
                    pcd_bbox.colors = o3d.utility.Vector3dVector(color_pcd_np)

                    pcd_bbox = pcd_bbox.random_down_sample(sampling_ratio=0.02)
                    labels = np.asarray(pcd_bbox.cluster_dbscan(eps=0.2, min_points=8, print_progress=False))
                    labels_count = np.bincount(labels + 1)
                    label_sort = np.argsort(labels_count)[::-1] - 1

                    # if (labels_count[label_max + 1] <= 50):
                    #     raise Exception("Too much small cluster!")
                    # label_second = label_sort[1]
                    # labels_flag_second = [True if a == label_second else False for a in labels]
                    # pcd_second = np.asarray(pcd_bbox.points).reshape([-1, 3])[labels_flag_second]

                    if label_sort[0] == -1:
                        label_max = label_sort[1]
                    else:
                        label_max = label_sort[0]

                    # label_max_ratio = labels_count[label_max - 1] / np.sum(labels_count)
                    # if(label_max_ratio>0.2): raise Exception("Too small cluster..(skip)")

                    labels_flag_max = [True if a == label_max else False for a in labels]
                    pcd_max = np.asarray(pcd_bbox.points).reshape([-1, 3])[labels_flag_max]

                    pcd_result_np = pcd_max

                    # pcd_max_mean = np.mean(pcd_max,axis=0)
                    # pcd_second_mean = np.mean(pcd_second,axis=0)
                    # sqrt(x*x + y*y + z*z)
                    # pcd_max_dist =  math.sqrt(pcd_max_mean[0] * pcd_max_mean[0] +
                    #                           pcd_max_mean[1] * pcd_max_mean[1] +
                    #                           pcd_max_mean[2] * pcd_max_mean[2])
                    #
                    # pcd_second_dist =  math.sqrt(pcd_second_mean[0] * pcd_second_mean[0] +
                    #                              pcd_second_mean[1] * pcd_second_mean[1] +
                    #                              pcd_second_mean[2] * pcd_second_mean[2])
                    # if (pcd_max_dist > pcd_second_dist):
                    #     pcd_result_np = pcd_second
                    # else:
                    #     pcd_result_np = pcd_max

                    pcd_result = o3d.geometry.PointCloud()
                    pcd_result_color = (np.asarray(self.colors[self.names.index(a[0])]) / 255.0)[::-1]

                    pcd_result.points = o3d.utility.Vector3dVector(pcd_result_np)
                    pcd_result.paint_uniform_color(pcd_result_color)

                    pcd_result_3d_bbox = pcd_result.get_axis_aligned_bounding_box()
                    pcd_result_3d_bbox.color = pcd_result_color

                    pcd_total += pcd_result
                    bbox_total.append(pcd_result_3d_bbox)

                except Exception as e:
                    print(f"Can't Find Dominant PCD!: {e}")
                    continue

        self.vis_o3d.pcd_show(pcd_total, bbox=bbox_total)
        # view(img)

        msg_str = std_msgs.msg.String()
        msg_str.data = "asd"
        self.pub_check.publish(msg_str)

        process_time = datetime.now() - dt0
        self.rate_pub.sleep()
        if(self.show_FPS):
            # print("FPS = {0}".format(1/process_time.total_seconds()))
            print("Running Time(ms) = {0:.3f}".format(process_time.total_seconds() * 1000.0))

    def print_info(self, msg: str):
        self.get_logger().info(msg)

    def unpack(self, s_arr: SerializedArray):
        type_arr = locate(f"numpy.{s_arr.type}")
        return np.frombuffer(s_arr.data, type_arr).reshape(s_arr.shape)

if __name__ == '__main__':

    device = '0'
    weight_path =  os.path.join(SHARE_DIR, 'weights/0911_last.pt')

    # Initialize
    set_logging()
    device = select_device(device)

    # Load model
    model = attempt_load(weight_path, map_location=device)  # load FP32 model

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # loop_cam(0)

    # ROS2
    rclpy.init(args=None)

    node = detection_node(model, device)
    rclpy.spin(node)
