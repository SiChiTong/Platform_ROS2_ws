#!/usr/bin/env python3
import math
import os

import std_msgs.msg
import sys
from ament_index_python.packages import get_package_share_directory
from xacro import process

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
import traceback
import cupoch.cupoch as cph
import open3d as o3d

from models.experimental import attempt_load
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, apply_classifier, check_file, check_img_size, check_imshow, check_requirements,
                           check_suffix, colorstr, increment_path, non_max_suppression, print_args, scale_coords,
                           strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import load_classifier, select_device, time_sync

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from pcd_manager.msg import RGBDImage, SerializedArray
from pydoc import locate
from datetime import datetime
from tf2_ros import Buffer, TransformListener
from detection_2d.msg import BoundingBox3D, DetectionResult
from visualization_msgs.msg import MarkerArray, Marker

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

def plot_one_box(x, img, color=None, label=None, line_thickness=3):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

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
    #print("yolo img:", img.shape)
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
    t1 = time_sync()
    pred = model(img)[0]

    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
    t2 = time_sync()

    # Visualization
    det = pred[0]
    im0 = img0[0].copy()
    #print("im0_shpape:", im0.shape)
    list_result = []

    if len(det):
        data1 = "Object detected! "
        data2 = "object classes : "
        # Rescale boxes from img_size to im0 size
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

        # Write results
        for *xyxy, conf, cls in reversed(det):
            c = int(cls)
            label = f'{names[c]} {conf:.2f}'
            plot_one_box(xyxy, im0, label=label, color=colors[c], line_thickness=1)            
            #annotator.box_label(xyxy, label, color=colors(c, True))
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
        #print(f'Done. ({t2 - t1:.3f}s)')
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

class cph_manager:
    def __init__(self, isVisualize=True):
        self.vis = cph.visualization.Visualizer()
        self.pointcloud = cph.geometry.PointCloud()
        self.isinit = False
        self.isVisualize = isVisualize
        self.bbox_prev = []

    def add_pcd_to_vis(self):
        if((not self.isinit) and (self.isVisualize)):
            self.vis.create_window("Test")
            mesh = cph.geometry.TriangleMesh.create_coordinate_frame(origin=[0.0, -0.85, 0.0])
            self.isinit = self.vis.add_geometry(self.pointcloud) and \
                          self.vis.add_geometry(mesh)

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
        result = []
        result.append(self.vis.update_geometry(self.pointcloud))
        for a in self.bbox_prev:
            result.append(self.vis.update_geometry(a))

        return result

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
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0.0, -0.85, 0.0])
            self.isinit = self.vis.add_geometry(self.pointcloud) and \
                          self.vis.add_geometry(mesh)

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

# class rate_mine():
#     def __init__(self, rate):
#         self.rate = rate
#         self.dt0 = datetime.now()
#
#     def sleep(self, node):
#         process_time = (datetime.now() - self.dt0).total_seconds()
#         period = 1.0 / self.rate
#
#         rclpy.spin_once(node, timeout_sec=0.01)
#         while (process_time <= period):
#             rclpy.spin_once(node, timeout_sec=0.01)
#             process_time = (datetime.now() - self.dt0).total_seconds()
#
#         self.dt0 = datetime.now()

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

node_namespace = "detect_yolo"

class detection_node(Node):
    def __init__(self, model, device):
        super().__init__(node_name="detector", namespace=node_namespace)

        self.declare_parameter("show_running_time", value=True)
        self.declare_parameter("rate", value=10.0)
        self.declare_parameter("downsample_rate", value=0.05)
        self.declare_parameter("use_open3d", value=False)
        self.declare_parameter("cam_frame_id", value="realsense_d455")
        self.declare_parameter("show_pcd", value=False)
        self.declare_parameter("show_cv2", value=True)

        self.show_FPS = self.get_parameter('show_running_time').value
        self.downsample_rate = self.get_parameter('downsample_rate').value
        self.rate_pub = self.get_parameter('rate').value
        self.use_o3d =  self.get_parameter('use_open3d').value
        self.cam_frame_id =  self.get_parameter('cam_frame_id').value
        self.show_pcd = self.get_parameter('show_pcd').value
        self.show_cv2 = self.get_parameter('show_cv2').value
        self.dt0 = datetime.now()

        self.buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.buffer, self)

        self.model = model

        self.names = model.module.names if hasattr(model, 'module') else model.names  # get class names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

        if self.use_o3d:
            self.vis = o3d_manager()
        else:
            self.vis = cph_manager()

        self.device = device

        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
            QoSLivelinessPolicy, QoSReliabilityPolicy

        rgbd_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            avoid_ros_namespace_conventions=False
        )

        print(self.default_callback_group)
        self.group_1 = MutuallyExclusiveCallbackGroup()
        self.group_2 = ReentrantCallbackGroup()

        self.sub_rgbd = self.create_subscription(RGBDImage, '/pcd_realsense', self.callback_rgbd, rgbd_profile,
                                                 callback_group=self.group_2)
        self.timer = self.create_timer((1.0/ self.rate_pub), self.callback_timer, callback_group=self.group_2)

        # self.sub_rgbd = self.create_subscription(RGBDImage, '/pcd_realsense', self.callback_rgbd, rgbd_profile)
        # self.timer = self.create_timer((1.0/ self.rate_pub), self.callback_timer)

        self.pub_bbox = self.create_publisher(DetectionResult,'/bbox',5)
        self.pub_marker = self.create_publisher(MarkerArray,'/bbox_markers',5)
        self.rgbd_img = None

        self.print_info("Success for Initialization!")

    def callback_rgbd(self, msg: RGBDImage):
        process_time = datetime.now() - self.dt0

        self.rgbd_img = msg
        self.dt0 = datetime.now()
        if(self.show_FPS):
            print("Sub Time(ms) = {0:.3f}".format(process_time.total_seconds() * 1000.0))

    def callback_timer(self):
        self.process(self.rgbd_img)

    def process(self,msg: RGBDImage):

        if msg == None:
            print("msg is None.")
            return

        img: np.ndarray = self.unpack(msg.color)
        pcd: np.ndarray = self.unpack(msg.depth_to_xyz)

        if self.use_o3d:
            pcd_total = o3d.geometry.PointCloud()
        else:
            pcd_total = cph.geometry.PointCloud()

        bbox_total = []
        detection_result = []

        with torch.no_grad():

            # result : [label, point_Left_Up, point_Right_Down, confidence]

            result = detect(self.device, self.model, img, self.names, self.colors,
                            imgsz=list(img.shape[0:2]), conf_thres=0.3, iou_thres=0.3, isVisualized=self.show_cv2)

            pcd_img = np.concatenate((pcd, img), axis=2)
            img_size = pcd_img.shape[0]*pcd_img.shape[1]

            for a in result:
                r1, r2 = a[1], a[2]
                pcd_img_bbox = pcd_img[r1[1]:r2[1], r1[0]:r2[0], ...]
                pcd_np = pcd_img_bbox.reshape([-1, 6])
                pcd_nan = ~np.isnan(pcd_np)[:,0]
                pcd_inf = ~np.isinf(pcd_np)[:,0]
                pcd_btm = np.where(pcd_np[:, 1] > -0.68, True, False)
                aac = np.logical_and(np.logical_and(pcd_nan, pcd_inf), pcd_btm)
                aab = pcd_img_bbox.reshape([-1, 6])[aac]

                try:
                    ratio_nan = (np.size(aac) - np.count_nonzero(aac)) / np.size(aac)
                    if(ratio_nan>0.8): raise Exception("Too many Nan value..(skip)")

                    pcd_size = aab.shape[0]
                    bbox_ratio = pcd_size / img_size
                    downsample_rate = self.downsample_rate

                    if(bbox_ratio > 0.3):
                        downsample_rate *= 0.25
                    elif(bbox_ratio > 0.2):
                        downsample_rate *= 0.4
                    elif (bbox_ratio > 0.1):
                        downsample_rate *= 0.7
                    elif (bbox_ratio < 0.05):
                        downsample_rate *= 1.5
                    elif (bbox_ratio < 0.02):
                        downsample_rate *= 3.0
                    elif (bbox_ratio < 0.01):
                        downsample_rate *= 5.0

                    if downsample_rate > 1: downsample_rate = 1

                    pcd_down = aab[np.random.choice(pcd_size, round(pcd_size * downsample_rate), replace=False)]
                    point_pcd_np = pcd_down[:, 0:3]
                    color_pcd_np = (pcd_down[:, 3:6] / 255.0)[..., ::-1]

                    if(self.use_o3d):
                        pcd_bbox = o3d.geometry.PointCloud()

                        pcd_bbox.points = o3d.utility.Vector3dVector(point_pcd_np)
                        pcd_bbox.colors = o3d.utility.Vector3dVector(color_pcd_np)
                    else:
                        pcd_bbox = cph.geometry.PointCloud()

                        pcd_bbox.points = cph.utility.Vector3fVector(point_pcd_np)
                        pcd_bbox.colors = cph.utility.Vector3fVector(color_pcd_np)

                    if(self.use_o3d):
                        labels = np.asarray(pcd_bbox.cluster_dbscan(eps=0.3, min_points=25, print_progress=False))
                    else:
                        labels = np.asarray(pcd_bbox.cluster_dbscan(eps=0.3, min_points=25, print_progress=False).cpu())

                    labels_count = np.bincount(labels + 1)
                    label_sort = np.argsort(labels_count)[::-1] - 1

                    if label_sort.tolist() == [-1]:
                        raise Exception("Not Find Any Cluster..(skip)")

                    if label_sort[0] == -1:
                        label_max = label_sort[1]
                    else:
                        label_max = label_sort[0]

                    # label_max_ratio = labels_count[label_max - 1] / np.sum(labels_count)
                    # if(label_max_ratio>0.2): raise Exception("Too small cluster..(skip)")

                    labels_flag_max = [True if a == label_max else False for a in labels]

                    if(self.use_o3d):
                        pcd_max = np.asarray(pcd_bbox.points).reshape([-1, 3])[labels_flag_max]
                    else:
                        pcd_max = np.asarray(pcd_bbox.points.cpu()).reshape([-1, 3])[labels_flag_max]

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

                    if(self.use_o3d):
                        pcd_result = o3d.geometry.PointCloud()
                        pcd_result_color = (np.asarray(self.colors[self.names.index(a[0])]) / 255.0)[::-1]

                        pcd_result.points = o3d.utility.Vector3dVector(pcd_result_np)
                        pcd_result.paint_uniform_color(pcd_result_color)

                        pcd_result_3d_bbox = pcd_result.get_axis_aligned_bounding_box()
                        pcd_result_3d_bbox.color = pcd_result_color
                    else:
                        pcd_result = cph.geometry.PointCloud()
                        pcd_result_color = (np.asarray(self.colors[self.names.index(a[0])]) / 255.0)[::-1]

                        pcd_result.points = cph.utility.Vector3fVector(pcd_result_np)
                        pcd_result.paint_uniform_color(pcd_result_color)

                        pcd_result_3d_bbox = pcd_result.get_axis_aligned_bounding_box()
                        pcd_result_3d_bbox.color = pcd_result_color

                    pcd_total += pcd_result
                    bbox_total.append(pcd_result_3d_bbox)
                    detection_result.append(a)

                except Exception as e:
                    print(f"Can't Find Dominant PCD!: {e}")
                    continue

        if self.show_pcd: self.vis.pcd_show(pcd_total, bbox=bbox_total)
        self.publish_bbox(bbox_total, detection_result)

    def print_info(self, msg: str):
        self.get_logger().info(msg)

    def unpack(self, s_arr: SerializedArray):
        type_arr = locate(f"numpy.{s_arr.type}")
        return np.frombuffer(s_arr.data, type_arr).reshape(s_arr.shape)

    def get_cam_base(self, target: str = "realsense_d455"):
        try:
            tf = self.buffer.lookup_transform("robot_base", target, Time())  # Blocking
            return tf
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            return None

    def publish_bbox(self, bbox_total: list, result: list):
        from std_msgs.msg import Header,ColorRGBA
        from geometry_msgs.msg import Point, Pose, Vector3
        if(bbox_total.__len__() == result.__len__()):
            result_msg = []
            markers = []
            for i in range(bbox_total.__len__()):
                bbox = bbox_total[i]
                result_bbox = result[i]

                maxb_x = float(-bbox.max_bound[2])
                maxb_y = float(-bbox.max_bound[0])
                maxb_z = float( bbox.max_bound[1])

                minb_x = float(-bbox.min_bound[2])
                minb_y = float(-bbox.min_bound[0])
                minb_z = float( bbox.min_bound[1])

                header = Header()
                header.frame_id = self.cam_frame_id
                header.stamp = self.get_clock().now().to_msg()

                color = (np.asarray(self.colors[self.names.index(result_bbox[0])]) / 255.0)[::-1]
                pose = Pose(position=Point(x=(maxb_x + minb_x) / 2, y=(maxb_y + minb_y) / 2, z=(maxb_z + minb_z) / 2))
                scale = Vector3(x=(maxb_x - minb_x) / 2, y=(maxb_y - minb_y) / 2, z=(maxb_z - minb_z) / 2)
                color_msg = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.8)

                from builtin_interfaces.msg import Duration

                lifetime = Duration(nanosec=100000000)

                if result_bbox[0] == "person":
                    msg = BoundingBox3D(name = result_bbox[0],confidence_score=result_bbox[3],
                                        max_bound=Point(x=maxb_x, y=maxb_y, z=maxb_z),
                                        min_bound=Point(x=minb_x, y=minb_y, z=minb_z),
                                        scale=scale, pose=pose, color=color_msg,
                                        header=header)

                    marker = Marker(header=header, ns="obstacles", id=i, type=Marker.CUBE,
                                    pose=pose, scale=scale, color=color_msg, text=result_bbox[0], lifetime=lifetime)
                    result_msg.append(msg)
                    markers.append(marker)

            self.pub_bbox.publish(DetectionResult(result=result_msg))
            self.pub_marker.publish(MarkerArray(markers=markers))

class pcd_listener(Node):
    def __init__(self, detector: detection_node):
        super().__init__(node_name="listener", namespace=node_namespace)
        self.detector = detector

        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
            QoSLivelinessPolicy, QoSReliabilityPolicy

        rgbd_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            avoid_ros_namespace_conventions=False
        )

        print(self.default_callback_group)
        self.group_1 = MutuallyExclusiveCallbackGroup()
        self.group_2 = ReentrantCallbackGroup()

        self.sub_rgbd = self.create_subscription(RGBDImage, '/pcd_realsense', self.callback_rgbd, rgbd_profile,
                                                 callback_group=self.group_2)
        # self.sub_rgbd = self.create_subscription(RGBDImage, '/pcd_realsense', self.callback_rgbd, rgbd_profile)
        # self.timer = self.create_timer((1.0/ self.rate_pub), self.callback_timer)

        self.rgbd_img = None
        self.dt0 = datetime.now()
        self.get_logger().info("Success for Initialze Listener!")

    def callback_rgbd(self, msg: RGBDImage):
        process_time = datetime.now() - self.dt0

        self.detector.rgbd_img = msg
        self.dt0 = datetime.now()
        if (self.detector.show_FPS):
            self.get_logger().info("Sub Time(ms) = {0:.3f}".format(process_time.total_seconds() * 1000.0))

if __name__ == '__main__':

    device = ''
    weight_path =  os.path.join(SHARE_DIR, 'weights/yolov5m6.pt')

    # Initialize
    # set_logging()
    device = select_device(device)

    # Load model
    model = attempt_load(weight_path, map_location=device)  # load FP32 model

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # ROS2
    rclpy.init(args=None)

    node_detector = detection_node(model, device)

    # node_listener = pcd_listener(node_detector)
    # executor = MultiThreadedExecutor(num_threads=None)
    # executor.add_node(node_detector)
    # executor.add_node(node_listener)
    # executor.spin()

    rclpy.spin(node_detector)



