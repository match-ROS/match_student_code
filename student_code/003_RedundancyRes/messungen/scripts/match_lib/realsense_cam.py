#!/usr/bin/env python
# coding=latin-1

import copy
import pyrealsense2 as rs
import rospy
import numpy as np
import cv2 as cv
import tf2_ros
import geometry_msgs.msg as geom_msg
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import json
import os
import math
import time

from match_geometry import MyPoint


class Camera(object):
    """class for handeling realsense depth camera

        Args:
            depth (bool, optional): activate depth stream. Defaults to True.
            color (bool, optional): activate color stream. Defaults to True.
            pathSettingsJson ([type], optional): path of json file with camera settings
            filter (bool, optional): activate different filters for depth stream. Defaults to True.
            alignTo (string, optional): Align to: "color" or "depth". Defaults to None.
            other_tf_trees (list[parent, child, trans=(0.0,0.0,0.0), rot=(0.0,0.0,0.0,1.0)]: 
        """

    def __init__(self, depth=True, color=True, pathSettingsJson=None, filter=True, alignTo=None, other_tf_trees=None):
        other_tf_msgs = [self._create_tf_msg(*tf) for tf in other_tf_trees] if other_tf_trees is not None else None
        self._cast_static_tfs(other_tf_msgs)
        self.filter = Filter()
        self.use_filter = filter

        self.stream_f = 30
        self.stream_h = 480
        self.stream_w = 848
        if pathSettingsJson is not None:
            json_obj = json.load(pathSettingsJson)
            try:
                self.stream_f = int(json_obj['viewer']['stream-fps'])
                self.stream_w = int(json_obj['viewer']['stream-width'])
                self.stream_h = int(json_obj['viewer']['stream-height'])
            except KeyError:
                print("Stream values not set. Using default")

        self.pipeline = rs.pipeline()

        # configure streams:
        config = rs.config()
        if depth:
            config.enable_stream(rs.stream.depth, self.stream_w, self.stream_h, rs.format.z16, self.stream_f)
        if color:
            config.enable_stream(rs.stream.color, self.stream_w, self.stream_h, rs.format.bgr8, self.stream_f)

        # Get intrinsics:
        profile = self.pipeline.start(config)
        self.dev = profile.get_device()
        self.intrinsics = self.get_intrinsics()

        # Get extrinsics:
        if color and depth:
            self.extrinsics = self.get_extrinsics()
        else:
            self.extrinsics = None

        # Get depth_scale:
        self.depth_scale = self.dev.first_depth_sensor().get_depth_scale()

        # Load Json (raise parameter disparity shift for near objects):
        if pathSettingsJson is not None:
            json_str = json.dumps(json_obj)
            advnc_mode = rs.rs400_advanced_mode(self.dev)
            advnc_mode.load_json(json_str)

        # align color or depth stream:
        if depth and color:
            if alignTo == "color":
                self._alignTo = rs.align(rs.stream.color)  # align depth to color stream
            elif alignTo == "depth":
                self._alignTo = rs.align(rs.stream.depth)  # align color to depth stream
            else:
                self._alignTo = None
                rospy.loginfo("No Alignment")
        else:
            self._alignTo = None

        # Vars to save frame data:
        self.color_data, self.depth_data = None, None

    def setROIFromPointAndSize(self, pos=(0.0, 0.0, 0.0), width=10, heigth=10):
        """Sets Region of interest from 3D-point and returns center of region in u,v

        Args:
            pos (tuple): Point to focus on. Defaults to (0.0,0.0,0.0).
            width (int, optional): width of ROI in pixel. Defaults to 10.
            heigth (int, optional): heigth of ROI in pixel. Defaults to 10.

        Returns:
            boolean: True if point is in view of camera
            List[float[2]]: center of ROI (point in coordinates of camera)
        """

        if pos[2] == 0:
            rospy.loginfo("Z is 0")
            return False, None

        # Get pixel
        px = rs.rs2_project_point_to_pixel(self.intrinsics["depth"], pos)

        if width > self.stream_w: width = self.stream_w
        if heigth > self.stream_h: heigth = self.stream_h

        maxX = int(px[0] + width / 2)
        minX = int(px[0] - width / 2)
        maxY = int(px[1] + heigth / 2)
        minY = int(px[1] - heigth / 2)

        if minX > self.stream_w or maxX > self.stream_w:
            minX = self.stream_w - width
            maxX = self.stream_w

        if minX < 0 or maxX < 0:
            minX = 0
            maxX = width

        if minY > self.stream_w or maxY > self.stream_w:
            minY = self.stream_w - heigth
            maxY = self.stream_w

        if minY < 0 or maxY < 0:
            minY = 0
            maxY = heigth

        self.set_roi(maxX, maxY, minX, minY)

        if 0 < px[0] < self.stream_w and 0 < px[1] < self.stream_h:
            return True, px
        else:
            return False, px

    def getROIFromPoint(self, pos=(0.0, 0.0, 0.0), width=40, heigth=40):
        """Sets Region of interest from 3D-point and returns center of region in u,v

        Args:
            pos (tuple): Point to focus on. Defaults to (0.0,0.0,0.0). Gets multiplied by cam.depth_scale
            width (int, optional): width of ROI in mm. Defaults to 10.
            heigth (int, optional): heigth of ROI in mm. Defaults to 10.

        Returns:
            boolean: True if point is in view of camera
            np.array[int[2]]: center of ROI (point in coordinates of camera)
            np.array[int[2], int[2]]: (px_min, px_max): ROI values
        """

        if pos[2] == 0:
            rospy.loginfo("Z is 0")
            return False, None, (None, None)
        pos = np.array(copy.copy(pos))
        pos /= self.depth_scale
        xy_diff = (int(width/2), int(heigth/2), 0)
        px_max = np.array(rs.rs2_project_point_to_pixel(self.intrinsics["depth"], pos + xy_diff))
        px_min = np.array(rs.rs2_project_point_to_pixel(self.intrinsics["depth"], pos - xy_diff))
        # Get pixel
        # px_max = np.array(rs.rs2_project_point_to_pixel(self.intrinsics["depth"], pos_max))
        # px_min = np.array(rs.rs2_project_point_to_pixel(self.intrinsics["depth"], pos_min))

        if width > self.stream_w: width = self.stream_w
        if heigth > self.stream_h: heigth = self.stream_h

        maxX = int(px_max[0])
        minX = int(px_min[0])
        maxY = int(px_max[1])
        minY = int(px_min[1])

        if minX > self.stream_w or maxX > self.stream_w:
            minX = self.stream_w - width
            maxX = self.stream_w

        if minX < 0 or maxX < 0:
            minX = 0
            maxX = width

        if minY > self.stream_w or maxY > self.stream_w:
            minY = self.stream_w - heigth
            maxY = self.stream_w

        if minY < 0 or maxY < 0:
            minY = 0
            maxY = heigth

        # self.set_roi(maxX, maxY, minX, minY)

        px_center = ((px_max+px_min)/2).astype('int')
        if 0 < px_center[0] < self.stream_w and 0 < px_center[1] < self.stream_h:
            return True, px_center, (px_min, px_max)
        else:
            return False, px_center, (px_min, px_max)

    def set_roi(self, maxX=None, maxY=None, minX=None, minY=None):
        """Sets region of interest for infrared cams

        Args:
            roi (pyrealsense2.pyrealsense2.region_of_interest): (maxX, maxY, minX, minY)
        """
        s = self.dev.first_roi_sensor()
        roi = s.get_region_of_interest()  # old roi
        # roi = rs.region_of_interest()
        if maxX is not None: roi.max_x = maxX
        if maxY is not None: roi.max_y = maxY
        if minX is not None: roi.min_x = minX
        if minY is not None: roi.min_y = minY
        # set
        s.set_region_of_interest(roi)

    def _cast_static_tfs(self, other_msgs=None):
        tf_caster = tf2_ros.StaticTransformBroadcaster()
        msgs = [self._create_tf_msg("camera_link", "camera_depth_frame"),
                self._create_tf_msg("camera_depth_frame", "camera_depth_optical_frame", rot=(-0.5, 0.5, -0.5, 0.5)),
                self._create_tf_msg("camera_link", "camera_color_frame",
                                    trans=(-0.000382322788937, 0.0148443710059, 0.000229349301662),
                                    rot=(0.002997583244, -0.00174734566826, 0.00472849281505, 0.999982774258)),
                self._create_tf_msg("camera_color_frame", "camera_color_optical_frame", rot=(-0.5, 0.5, -0.5, 0.5))]
        if other_msgs is not None:
            msgs.append(*other_msgs)
        tf_caster.sendTransform(msgs)

    def _create_tf_msg(self, parent, child, trans=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0)):
        tf_msg = geom_msg.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = trans[0]
        tf_msg.transform.translation.y = trans[1]
        tf_msg.transform.translation.z = trans[2]
        tf_msg.transform.rotation.x = rot[0]
        tf_msg.transform.rotation.y = rot[1]
        tf_msg.transform.rotation.z = rot[2]
        tf_msg.transform.rotation.w = rot[3]
        return tf_msg

    def release(self):
        """[Stops pipeline]
        """
        self.pipeline.stop()

    def get_frames_data(self):  # TODO: ggf frames zwischenspeichern: if new=False kein waitforframes
        """[get tuple (res, color_data, depth_data)]

        Returns:
            [bool]: [result]
            [pyrealsense2.BufData] : [color]
            [pyrealsense2.BufData] : [depth]
        """

        color_frame, depth_frame = self.get_frames()

        depth_data, color_data = None, None
        if depth_frame:
            depth_data = depth_frame.get_data()
        if color_frame:
            color_data = color_frame.get_data()
        elif not depth_frame:
            # If no frame:
            print("No frame, Camera connected?")
            return False, None, None

        self.color_data, self.depth_data = color_data, depth_data
        return True, color_data, depth_data

    def get_imgs(self):
        """[get tuple (res, color_img, depth_img)]

        Returns:
            [bool]: [result]
            [np.array] : [color_img as 2-dim-array]
            [np.array] : [depth as 2-dim-array]
        """
        res, color, depth = self.get_frames_data()
        return res, np.asanyarray(color), np.asanyarray(depth)

    def get_color_img(self):
        """returns depth as image

        Returns:
            [np.array]: [depth as 2-dim-array]
        """
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("No frame, Camera connected?")
            return None
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_depth_img(self):
        """returns depth as image

        Returns:
            [np.array]: [depth as 2-dim-array]
        """
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            print("No frame, Camera connected?")
            return None
        if self.use_filter:
            depth_frame = self.filter.filter(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image

    def get_depth_img_normal(self, max_thres=None, min_thres=None, depth_img=None):
        """returns depth normalized to uint8

        Args:
            max_thres (int, optional): maximum distance. Defaults to None.
            min_thres (int, optional): minimum distance. If below: val=0 Defaults to None.
            depth_img (np.array, optional): img to normalize. If None: get img from cam

        Returns:
            np.array8: depth img as 2-dim array scaled to max value 254
            np.array: depth img with depth in mm as 2-dim-array
            float: scale factor (maxDistance/254)
        """
        if depth_img is None:
            depth_img = self.get_depth_img()
        if max_thres is not None:
            depth_img[depth_img > max_thres] = 0
        if min_thres is not None:
            # depth_img = (min_thres <= depth_img)*depth_img
            depth_img[depth_img < min_thres] = 0
        try:
            scale_fac = 254.0 / depth_img.max()
        except ZeroDivisionError:
            scale_fac = 0  # should not happen often. If ZeroDev on regular basis: if depth_img.max() .../else 0
        depth_img_thres8 = np.uint8(cv.normalize(depth_img, None, 0, 254, cv.NORM_MINMAX))

        return depth_img_thres8, depth_img, scale_fac

    def get_frames(self):
        """returns (aligned) frames

        Returns:
            color_frame: aligned if self._alignTo is set
            depth_frame: aligned if self._alignTo is set
        """
        frames = self.pipeline.wait_for_frames()

        return self.align_frames(frames)

    def align_frames(self, frames):
        """Aligns frames if align is set

        Returns:
            color_frame, depth_frame
        """
        if self._alignTo is not None:
            aligned_frames = self._alignTo.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
        else:
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

        return color_frame, depth_frame

    def get_intrinsics_depth(self):
        """returns the intrinsic parameters of depth stream

        Returns:
            intrinsics: i.e.: [ 640x480  p[318.129 242.52]  f[383.523 383.523]  Brown Conrady [0 0 0 0 0] ]
        """
        try:
            intrin = self.pipeline.get_active_profile().get_stream(
                rs.stream.depth).as_video_stream_profile().get_intrinsics()
        except RuntimeError:
            rospy.loginfo("No intrinsics for depth stream")
            intrin = None
        return intrin

    def get_intrinsics_color(self):
        """returns the intrinsic parameters of depth stream

        Returns:
            intrinsics: i.e.: [ 640x480  p[318.129 242.52]  f[383.523 383.523]  Brown Conrady [0 0 0 0 0] ]
        """
        try:
            intrin = self.pipeline.get_active_profile().get_stream(
                rs.stream.color).as_video_stream_profile().get_intrinsics()
        except RuntimeError:
            rospy.loginfo("No intrinsics for color stream")
            intrin = None
        return intrin

    def get_intrinsics(self):
        """Returns intrinsics of depth and color

        Returns:
            intrinsics (dict): {depth, color}
        """
        intrins = {"depth": self.get_intrinsics_depth(), "color": self.get_intrinsics_color()}
        return intrins

    def get_extrinsics(self):
        """Returns extrinsics between depth and color

        Returns:
            extrinsics (dict): {depth_to_color, color_to_depth}
        """
        profile = self.pipeline.get_active_profile()
        depth_to_color_extrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(
            profile.get_stream(rs.stream.color))
        color_to_depth_extrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(
            profile.get_stream(rs.stream.depth))
        extrinsics = {"depth_to_color": depth_to_color_extrin, "color_to_depth": color_to_depth_extrin}
        return extrinsics


class Filter(object):
    """Filter obj for applying different filters to depth frame (spatial, temporal, hole_filling)
    """

    def __init__(self):
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)

        #  preserving z-accuracy and performing some rudamentary hole-filling (lower spatial resolution)
        self.decimation = rs.decimation_filter()
        # self.decimation.set_option(rs.option.filter_magnitude, 4)

        # Domain-Transform Edge Preserving Smoothing
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 5)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 1)
        self.spatial.set_option(rs.option.filter_smooth_delta, 50)
        self.spatial.set_option(rs.option.holes_fill, 3)

        # basic temporal smoothing and hole-filling (several frames needed)
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()

    def filter(self, frame):
        # frame = self.decimation.process(frame)
        frame = self.depth_to_disparity.process(frame)  #ggf gleich disparity aus ir streams?
        frame = self.spatial.process(frame)
        frame = self.temporal.process(frame)
        frame = self.disparity_to_depth.process(frame)
        frame = self.hole_filling.process(frame)
        return frame


def linesTo3dPoints(lines, depth, intrinsics):
    """calculates 3d points represented by end and beginning of lines

    Args:
        lines (array([[[x1,y1,x2,y2]],...])): start/end of lines
        depth (2-dim-array): real depth. if uint8-depth: depth*scale_factor
        intrinsics: realsense_cam.Camera.get_intrinsics_depth()

    Returns:
        np.array(array(x,y,z)): array of points in 3D (converted from mm to m)
    """
    # TODO: find nearest pixel with value > 0 or minD<value<maxD or line-direction to the inside (center)
    points = []
    if lines is None: return points

    for line in lines:
        x1, y1, x2, y2 = line[0]
        x1, y1, x2, y2 = int(x1) - 1, int(y1) - 1, int(x2) - 1, int(y2) - 1
        z1 = findValueBiggerThen(depth, [y1, x1])
        z2 = findValueBiggerThen(depth, [y2, x2])
        if z1:
            p1 = rs.rs2_deproject_pixel_to_point(intrinsics, (x1, y1), z1)
            points.append(p1)
        if z2:
            p2 = rs.rs2_deproject_pixel_to_point(intrinsics, (x2, y2), z2)  # *scale_fac
            points.append(p2)
    point_arr = np.array(points)
    point_arr /= 1000.0  # mm to m
    return point_arr


def points2DTo3D(depth, points_in, intrinsics):
    """calculates 3D points

    Args:
        depth (2-dim-array): real depth. if uint8-depth: depth*scale_factor
        points_in ([int, int]): pixel value for depth
        intrinsics realsense_cam.Camera.get_intrinsics_depth()

    Returns:
        np.array(array(x,y,z)): array of points in 3D (converted from mm to m)
    """
    points = []
    if points_in is None: return points

    for point in points_in:
        x, y = point[0]
        _, z = findValueBiggerThen(depth, [y, x])
        if z:
            p = rs.rs2_deproject_pixel_to_point(intrinsics, (x, y), z)
            points.append(p)
    point_arr = np.array(points)
    point_arr /= 1000.0  # mm to m
    return point_arr


def calc3DPoints(depth, intrinsics):
    """Calculates Points from depth img

    Args:
        depth (2-dim-array): real depth. if uint8-depth: depth*scale_factor
        intrinsics realsense_cam.Camera.get_intrinsics_depth()

    Returns:
        np.array(array(x,y,z)): array of points in 3D (converted from mm to m)
    """
    points = []
    shape = depth.shape
    for i in range(shape[0]):
        for j in range(shape[1]):
            z = depth[i, j]
            if z:
                p = rs.rs2_deproject_pixel_to_point(intrinsics, (j, i), z)
                points.append(p)
    point_arr = np.array(points)
    point_arr /= 1000.0  # mm to m
    return point_arr

    # pc = rs.pointcloud()
    # points_realsense = pc.calculate(cam.get_frames()[1])
    # points_all = points_realsense.get_vertices()
    # cloud_points = pcl2.create_cloud_xyz32(header,points_all)


def findValueBiggerThen(img, xy, minVal=0, max_delta=20):
    """returns value from img if bigger then minVal

    Args:
        img ([type]): img data where to search
        xy ([int, int]): pixel location
        minVal (int, optional): Defaults to 0.
        max_delta (int, optional): search radius. Defaults to 20.

    Returns:
        [x,y], val
    """

    def getVal(px):
        try:
            val = img[int(px[1]), int(px[0])]
        except IndexError:
            val = 0
        return val

    idx_delta = np.array([[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])
    xy = np.array(xy)
    val = 0

    i = 1
    while i <= max_delta:
        for delta in idx_delta:
            px = xy + delta * i

            val = getVal(px)
            if val != 0:
                return px, val
        i += 1
    return [0, 0], 0


if __name__ == '__main__':
    tf_extra = [["map", "camera_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]]

    rospy.init_node("Realsense_Camera_Module")
    cam = Camera(
        pathSettingsJson=open(os.path.abspath(os.path.dirname(__file__)) + "/camera-settings_highAccuracy.json"),
        other_tf_trees=tf_extra)
    print(cam.get_intrinsics_depth())

    # pos = MyPoint((0.11713785, -0.01958524, 0.253))
    pos = MyPoint((0.1, -1.0, -0.000000001))
    # res = cam.setROIFromPoint(pos.asArray[:3])
    # if not res:
    #     print("Point not in view")

    # else:
    #     print("pixel coordinates: ")
    #     print(res)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
