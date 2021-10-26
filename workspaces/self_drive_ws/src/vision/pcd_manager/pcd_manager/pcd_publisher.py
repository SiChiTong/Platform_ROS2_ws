#!/usr/bin/env python3

import numpy as np
import pyrealsense2 as rs
import time
import cv2
import json
import os
from datetime import datetime

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

        self.show_FPS = self.get_parameter('show_running_time').value
        self.rate_pub = self.get_parameter('rate').value
        self.init_realsense()
        self.pub_rgbd = self.create_publisher(RGBDImage, "/pcd_realsense", 10)

    def init_realsense(self, serial_dev='105322251720', json_path=os.path.join(SHARE_DIR, "config", "config_d455.json")):
        ctx = rs.context()
        devices = ctx.query_devices()
        devices_dic = {
                       a.get_info(rs.camera_info.serial_number):
                       {
                           "instance" : a,
                           "name" : a.get_info(rs.camera_info.name),
                       }
                       for a in devices}

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
            # ab = ~np.isnan(depth_to_xyz.reshape([-1, 3]))[:,0]
            # aab = depth_to_xyz.reshape([-1, 3])[ab]
            #
            # pcd_o3d = o3d.geometry.PointCloud()
            # pcd_o3d.points = o3d.utility.Vector3dVector(aab)
            #
            # self.pointcloud.points = pcd_o3d.points
            # self.pointcloud.colors = pcd_o3d.colors
            # if vis: self.pcd_show(pcd_o3d)

        return depth_to_xyz

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
    # Configure depth and color streams
    rclpy.init(args=None)

    node = pcd_manager_node()

    is_o3d_imported = False

    try:
        import open3d as o3d
        is_o3d_imported = True
    except ModuleNotFoundError:
        print("open3d module not found!")

    # Start streaming
    node.start_pipeline()
    pipeline = node.pipe_d455

    depth_profile = rs.video_stream_profile(node.profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    # Processing blocks
    pc = rs.pointcloud()
    align = rs.align(rs.stream.color)

    filters = [
                # rs.decimation_filter(magnitude=2.0),
                rs.hdr_merge(),
                rs.disparity_transform(True),
                # rs.spatial_filter(),
                rs.temporal_filter(smooth_alpha=0.25, smooth_delta=50.0, persistence_control=1),
                rs.disparity_transform(False),
              ]

    colorizer = rs.colorizer()
    curTime = prevTime = time.time()

    if is_o3d_imported:
        pcd_manager_o3d = o3d_manager(isVisualize=True)

    rate = rate_mine(node.rate_pub)

    while True:
        dt0 = datetime.now()
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        for filter_rs in filters:
            depth_frame = filter_rs.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

        depth_image: np.ndarray = np.asanyarray(depth_frame.get_data())
        color_image: np.ndarray = np.asanyarray(color_frame.get_data())
        # color_image: np.ndarray = np.asarray(color_frame.get_data())[..., ::-1]
        # color_image = np.asarray(color_image,order='C')

        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # depth_to_xyz = pcd_manager_o3d.rgbd_to_pcd(depth_intrinsics, depth_image, color_image, valid_only=True, vis=True)
        depth_to_xyz = pcd_manager_o3d.rgbd_to_pcd(depth_intrinsics, depth_image, color_image, vis=False)

        node.publish_pcd(depth_to_xyz, color_image)
        process_time = datetime.now() - dt0

        rate.sleep()
        if node.show_FPS:
            print("Running Time(ms) = {0:.3f}".format(process_time.total_seconds() * 1000.0))

        # ----------------------------------------- cv2 ---------------------------------------- #
        cv2_show_only_depth(depth_colormap)
