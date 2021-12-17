#!/usr/bin/env python3

import numpy as np

try:
    import pyrealsense2 as rs
except:
    import pyrealsense2.pyrealsense2 as rs

import time
import cv2
import json
import os
import cupoch.cupoch as cph
import open3d as o3d

from datetime import datetime
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

import rclpy

from rclpy.node import Node
from pcd_manager.msg import SerializedArray, RGBDImage
import builtin_interfaces.msg

from ament_index_python.packages import get_package_share_directory
SHARE_DIR = get_package_share_directory('pcd_manager')

class pcd_manager_node(Node):
    def __init__(self):
        super().__init__('pcd_manager_node')
        self.declare_parameter("show_running_time", value=True)
        self.declare_parameter("rate", value=20.0)
        self.declare_parameter("is_downsample", value=True)
        self.declare_parameter("cutoff_border", value=0.05)
        self.declare_parameter("show_pcd", value=True)
        self.declare_parameter("show_cv2", value=True)

        self.show_FPS = self.get_parameter('show_running_time').value
        self.rate_pub = self.get_parameter('rate').value
        self.is_downsample = self.get_parameter('is_downsample').value
        self.cutoff = self.get_parameter('cutoff_border').value        
        if self.cutoff == 0.0: self.cutoff = None        
        
        self.show_pcd = self.get_parameter('show_pcd').value
        self.show_cv2 = self.get_parameter('show_cv2').value

        self.init_realsense()
        self.start_pipeline()

        # self.pcd_manager = o3d_manager(isVisualize=True)
        self.pcd_manager = cph_manager(isVisualize=True)

        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
            QoSLivelinessPolicy, QoSReliabilityPolicy

        rgbd_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.SYSTEM_DEFAULT,
            avoid_ros_namespace_conventions=False
        )

        self.pub_rgbd = self.create_publisher(RGBDImage, "/pcd_realsense", 3)
        self.timer = self.create_timer(1.0 / self.rate_pub, self.callback_timer)

        self.buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.buffer, self)

    def init_realsense(self, serial_dev='105322251721', json_path=os.path.join(SHARE_DIR, "config", "high_density.json")):
        ctx = rs.context()
        devices = ctx.query_devices()
        devices_dic = {}
        for i in range(len(devices)):
            try:
                a = devices[i]
                serial_num = a.get_info(rs.camera_info.serial_number)
                name = a.get_info(rs.camera_info.name)
                devices_dic[serial_num] = {"instance": a, "name": name}
            except: continue

        if not serial_dev in devices_dic:
            print(f"can't find \"{serial_dev}\" device!")
            return

        # Intel RealSense D455
        try:
            device = devices_dic[serial_dev]["instance"]
            self.pipe_d455 = rs.pipeline()
            self.cfg_d455 = rs.config()
            self.cfg_d455.enable_device(serial_dev)
            self.cfg_d455.enable_stream(rs.stream.depth, rs.format.z16, 60)
            self.cfg_d455.enable_stream(rs.stream.color, rs.format.bgr8, 60)
            self.profile : rs.pipeline_profile = self.cfg_d455.resolve(self.pipe_d455)

            self.adv_mode = rs.rs400_advanced_mode(device)
            jsonObj = json.load(open(json_path))
            json_string= str(jsonObj).replace("'", '\"')

            self.adv_mode.load_json(json_string)

        except Exception as e:
            print(f"Can't load Intel RealSense D455 : {e}")
            self.pipe_d455 = None

        node = self

        depth_profile = rs.video_stream_profile(node.profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks
        self.align = rs.align(rs.stream.color)
        self.filters = []

        if node.is_downsample: self.filters.append(rs.decimation_filter(magnitude=2.0))

        self.filters.append(rs.hdr_merge())
        self.filters.append(rs.disparity_transform(True))
        self.filters.append(rs.spatial_filter(magnitude=2.0, smooth_alpha=0.45, smooth_delta=21.0, hole_fill=2))
        self.filters.append(rs.temporal_filter(smooth_alpha=0.25, smooth_delta=50.0, persistence_control=1))
        self.filters.append(rs.disparity_transform(False))

        self.colorizer = rs.colorizer()

    def start_pipeline(self):
        if self.pipe_d455 is not None:
            self.pipe_d455.start(self.cfg_d455)

    def publish_pcd(self, pcd: np.ndarray, img: np.ndarray):
        msg_pcd = SerializedArray()
        msg_pcd.shape = pcd.shape
        msg_pcd.data.frombytes(pcd.tobytes())
        msg_pcd.type = str(pcd.dtype)

        msg_img = SerializedArray()
        msg_img.shape = img.shape
        msg_img.data.frombytes(img.tobytes())
        msg_img.type = str(img.dtype)

        msg = RGBDImage()
        msg.depth_to_xyz = msg_pcd
        msg.color = msg_img

        self.pub_rgbd.publish(msg)

    def callback_timer(self):
        dt0 = datetime.now()
        node = self
        colorizer = self.colorizer
        pcd_manager = self.pcd_manager

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipe_d455.wait_for_frames()

        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        for filter_rs in self.filters:
            depth_frame = filter_rs.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

        depth_image: np.ndarray = np.asanyarray(depth_frame.get_data())
        color_image: np.ndarray = np.asanyarray(color_frame.get_data())

        depth_h, depth_w = depth_image.shape
        color_h, color_w, _ = color_image.shape

        ratio_w = color_w / depth_w

        if (ratio_w != 1):
            # color size > depth size
            if (ratio_w > 1):
                color_image = cv2.resize(color_image, [depth_w, depth_h])
            else:
                depth_image = cv2.resize(depth_image, [color_w, color_h])

        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        if(node.cutoff != None):
            img_min = round(depth_colormap.shape[1] * node.cutoff)
            img_max = depth_colormap.shape[1] - img_min
            depth_to_xyz = pcd_manager.rgbd_to_pcd(depth_intrinsics, depth_image, color_image, vis=node.show_pcd,
                                                   cutoff=node.cutoff)
            depth_colormap = depth_colormap[:, img_min:img_max]
            color_image = color_image[:, img_min:img_max]

            node.publish_pcd(depth_to_xyz, color_image)
        else:
            depth_to_xyz = pcd_manager.rgbd_to_pcd(depth_intrinsics, depth_image, color_image, vis=node.show_pcd,
                                                   cutoff=node.cutoff)
            node.publish_pcd(depth_to_xyz, color_image)

        # if (node.show_cv2): cv2_show_only_depth(depth_colormap)
        if (node.show_cv2): cv2_show(depth_colormap, color_image)

        process_time = datetime.now() - dt0
        if node.show_FPS:
            node.get_logger().info("Running Time(ms) = {0:.3f}".format(process_time.total_seconds() * 1000.0))

    def format_dt(self, t1: builtin_interfaces.msg.Time, t2: builtin_interfaces.msg.Time):
        """ Helper for formatting the difference between two stamps in microseconds """
        us1 = t1.sec * 1e6 + t1.nanosec // 1e3
        us2 = t2.sec * 1e6 + t2.nanosec // 1e3
        return f"{int(us2 - us1):5} [us]"

class o3d_manager:
    def __init__(self, isVisualize=True):
        self.vis = o3d.visualization.Visualizer()
        self.pointcloud = o3d.geometry.PointCloud()
        self.isinit = False
        self.isVisualize = isVisualize

    def add_pcd_to_vis(self):
        if((not self.isinit) and (self.isVisualize)):
            self.vis.create_window("Test")
            self.isinit = self.vis.add_geometry(self.pointcloud)

    def pcd_show(self, pcd):
        self.copy_pcd(pcd)
        self.add_pcd_to_vis()

        self.vis.update_geometry(self.pointcloud)
        self.vis.poll_events()
        self.vis.update_renderer()

    def copy_pcd(self, pcd):
        self.pointcloud.points = pcd.points
        self.pointcloud.colors = pcd.colors

    def rgbd_to_pcd(self, depth_intrinsics, depth_img: np.ndarray, color_img: np.ndarray, valid_only=False, vis=False):
        w, h = depth_intrinsics.width, depth_intrinsics.height

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height, depth_intrinsics.fx,
                                                                     depth_intrinsics.fy, depth_intrinsics.ppx, depth_intrinsics.ppy)

        open_img_depth = o3d.geometry.Image(depth_img)
        open_img_color = o3d.geometry.Image(color_img)
        # open_img_color = o3d.geometry.Image(np.asarray(color_img[...,::-1],order='C'))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(open_img_color, open_img_depth,
                                                                  convert_rgb_to_intensity=True)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic,
                                                             extrinsic=[[1, 0, 0, 0],
                                                                        [0, -1, 0, 0],
                                                                        [0, 0, -1, 0],
                                                                        [0, 0, 0, 1]],
                                                             project_valid_depth_only=valid_only)

        if(valid_only):
            depth_to_xyz = np.asarray(pcd.points).reshape([-1, 3])
            if vis: self.pcd_show(pcd)
        else:
            depth_to_xyz = np.asarray(pcd.points).reshape([h, w, -1])

        return depth_to_xyz

    def create_pcd(self, point, color):
        pcd = o3d.geometry.PointCloud()
        pcd.points = point
        pcd.colors = color
        return pcd

class cph_manager:
    def __init__(self, isVisualize=True):
        self.vis = cph.visualization.Visualizer()
        self.pointcloud = cph.geometry.PointCloud()
        self.isinit = False
        self.isVisualize = isVisualize

    def add_pcd_to_vis(self):
        if((not self.isinit) and (self.isVisualize)):
            self.vis.create_window("Test")
            mesh = cph.geometry.TriangleMesh.create_coordinate_frame(origin=[0.0, -0.85, 0.0])
            self.isinit = self.vis.add_geometry(self.pointcloud) and \
                          self.vis.add_geometry(mesh)

    def pcd_show(self, pcd):
        self.copy_pcd(pcd)
        self.add_pcd_to_vis()

        self.vis.update_geometry(self.pointcloud)
        self.vis.poll_events()
        self.vis.update_renderer()

    def copy_pcd(self, pcd):
        self.pointcloud.points = pcd.points
        self.pointcloud.colors = pcd.colors

    def rgbd_to_pcd(self, depth_intrinsics, depth_img: np.ndarray, color_img: np.ndarray, valid_only=False, vis=False,
                    cutoff=0.1):
        w, h = depth_intrinsics.width, depth_intrinsics.height

        pinhole_camera_intrinsic = cph.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height, depth_intrinsics.fx,
                                                                     depth_intrinsics.fy, depth_intrinsics.ppx, depth_intrinsics.ppy)

        open_img_depth = cph.geometry.Image(depth_img)
        open_img_color = cph.geometry.Image(color_img)

        rgbd = cph.geometry.RGBDImage.create_from_color_and_depth(open_img_color, open_img_depth, depth_scale=1000.0, depth_trunc=10.0,
                                                                  convert_rgb_to_intensity=False)

        pcd = cph.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic,
                                                             extrinsic=[[1, 0, 0, 0],
                                                                        [0, -1, 0, 0],
                                                                        [0, 0, -1, 0],
                                                                        [0, 0, 0, 1]],
                                                             project_valid_depth_only=valid_only)

        if(valid_only):
            depth_to_xyz = np.asarray(pcd.points.cpu()).reshape([-1, 3])
            if vis: self.pcd_show(pcd)
        else:
            depth_to_xyz = np.asarray(pcd.points.cpu()).reshape([h, w, -1])

            if(cutoff != None):
                img_min = round(depth_img.shape[1] * cutoff)
                img_max = depth_img.shape[1] - img_min
                depth_to_xyz = depth_to_xyz[:,img_min:img_max]

                if(vis):
                    color = color_img[:, img_min:img_max].reshape([-1, 3])
                    point = depth_to_xyz.reshape([-1, 3])

                    pnt_img = np.concatenate((point, color), axis=1)

                    ab = ~np.isnan(point)[:,0]
                    ac = ~np.isinf(point)[:,0]
                    aac = np.logical_and(ab,ac)
                    pnt_img = pnt_img[aac]
                    pcd = self.create_pcd(pnt_img[:,0:3],pnt_img[:,3:6])
                    self.pcd_show(pcd)

        return depth_to_xyz

    def create_pcd(self, point, color):
        pcd = cph.geometry.PointCloud()

        color = (color / 255.0)[..., ::-1]
        pcd.points = cph.utility.Vector3fVector(point)
        pcd.colors = cph.utility.Vector3fVector(color)
        return pcd

def cv2_show_only_depth(depth_img):
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', depth_img)
    cv2.waitKey(1)

def cv2_show(depth_color,color):
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    depth_colormap_dim = depth_color.shape
    color_colormap_dim = color.shape

    # If depth and color resolutions are different, resize color image to match depth image for display
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_color))
    else:
        images = np.hstack((color, depth_color))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)

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

if __name__ == "__main__":
    rclpy.init(args=None)
    node = pcd_manager_node()
    rclpy.spin(node)
