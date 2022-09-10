######################################################################
# File: utils.py
# Project: Research Project (M.Sc.)
# Created Date: 02.09.2022
# Author: Nico Leuze,
#         Faculty of Electrical Engineering and Information Technology
#         University of Applied Science Munich (MUAS)
# -----
# Last Modified: 02.09.2022
# Modified By: Nico Leuze
# -----
# Copyright (c) 2022 MUAS
######################################################################
import geometry_msgs.msg
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import scipy.spatial.transform
import std_msgs.msg
import tf2_ros



def to_cloud_msg(points, intensities=None, frame=None, stamp=None):
    """Convert list of unstructured points to a PointCloud2 message.

    Args:
        points: Point coordinates as array of shape (N,3).
        colors: Colors as array of shape (N,3).
        frame
        stamp
    """
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp or rospy.Time.now()

    msg.height = 1
    msg.width = points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = False

    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg.point_step = 12
    data = points

    if intensities is not None:
        msg.fields.append(PointField("intensity", 12, PointField.FLOAT32, 1))
        msg.point_step += 4
        data = np.hstack([points, intensities])

    msg.row_step = msg.point_step * points.shape[0]
    msg.data = data.astype(np.float32).tostring()

    return msg

def to_point_msg(position):
    """Convert numpy array to a Point message."""
    msg = geometry_msgs.msg.Point()
    msg.x = position[0]
    msg.y = position[1]
    msg.z = position[2]
    return msg


def from_point_msg(msg):
    """Convert a Point message to a numpy array."""
    return np.r_[msg.x, msg.y, msg.z]


def to_vector3_msg(vector3):
    """Convert numpy array to a Vector3 message."""
    msg = geometry_msgs.msg.Vector3()
    msg.x = vector3[0]
    msg.y = vector3[1]
    msg.z = vector3[2]
    return msg


def from_vector3_msg(msg):
    """Convert a Vector3 message to a numpy array."""
    return np.r_[msg.x, msg.y, msg.z]


def to_quat_msg(orientation):
    """Convert a `Rotation` object to a Quaternion message."""
    quat = orientation.as_quat()
    msg = geometry_msgs.msg.Quaternion()
    msg.x = quat[0]
    msg.y = quat[1]
    msg.z = quat[2]
    msg.w = quat[3]
    return msg


def from_quat_msg(msg):
    """Convert a Quaternion message to a Rotation object."""
    return Rotation.from_quat([msg.x, msg.y, msg.z, msg.w])


def to_pose_msg(transform):
    """Convert a `Transform` object to a Pose message."""
    # x = transform[0].as_quat()
    msg = geometry_msgs.msg.Pose()
    msg.position = to_point_msg(transform[1])
    msg.orientation = to_quat_msg(transform[0])
    return msg


def to_transform_msg(transform):
    """Convert a `Transform` object to a Transform message."""
    msg = geometry_msgs.msg.Transform()
    msg.translation = to_vector3_msg(transform.translation)
    msg.rotation = to_quat_msg(transform.rotation)
    return msg


def from_transform_msg(msg):
    """Convert a Transform message to a Transform object."""
    translation = from_vector3_msg(msg.translation)
    rotation = from_quat_msg(msg.rotation)
    return Transform(rotation, translation)


def to_color_msg(color):
    """Convert a numpy array to a ColorRGBA message."""
    msg = std_msgs.msg.ColorRGBA()
    msg.r = color[0]
    msg.g = color[1]
    msg.b = color[2]
    msg.a = color[3] if len(color) == 4 else 1.0
    return msg

def create_marker_msg(marker_type, frame, pose, scale, color):
    msg = Marker()
    msg.header.frame_id = frame
    msg.header.stamp = rospy.Time()
    msg.type = marker_type
    msg.action = Marker.ADD
    msg.pose = to_pose_msg(pose)
    msg.scale = to_vector3_msg(scale)
    msg.color = to_color_msg(color)
    return msg

def create_vol_msg(vol, voxel_size=0.0075, threshold=0.01):
    vol = vol.squeeze()
    points = np.argwhere(vol > threshold) * voxel_size
    values = np.expand_dims(vol[vol > threshold], 1)
    return to_cloud_msg(points, values, frame="voxel_grid_origin")

class Rotation(scipy.spatial.transform.Rotation):
    @classmethod
    def identity(cls):
        return cls.from_quat([0.0, 0.0, 0.0, 1.0])


def workspace_lines(dst):
    return [[0.0, 0.0, 0.0], [dst, 0.0, 0.0], [dst, 0.0, 0.0],
            [dst, dst, 0.0], [dst, dst, 0.0], [0.0, dst, 0.0],
            [0.0, dst, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, dst],
            [dst, 0.0, dst], [dst, 0.0, dst], [dst, dst, dst],
            [dst, dst, dst], [0.0, dst, dst], [0.0, dst, dst],
            [0.0, 0.0, dst], [0.0, 0.0, 0.0], [0.0, 0.0, dst],
            [dst, 0.0, 0.0], [dst, 0.0, dst], [dst, dst, 0.0],
            [dst, dst, dst], [0.0, dst, 0.0], [0.0, dst, dst]]

