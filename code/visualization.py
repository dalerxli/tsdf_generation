######################################################################
# File: visualization.py
# Project: Research Project (M.Sc.)
# Created Date: 02.09.2022
# Ownership: Code is largerly based on (VGN, Breyer et al., 2020)
# Author: Nico Leuze,
#         Faculty of Electrical Engineering and Information Technology
#         University of Applied Science Munich (MUAS)
# -----
# Last Modified: 02.09.2022
# Modified By: Nico Leuze
# -----
# Copyright (c) 2022 MUAS
######################################################################
import rospy, scipy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import utils
import numpy as np

class Visualization_Rviz(object):

    def __init__(self):
        rospy.init_node('node_tsdf_generation', anonymous=False)
        self.DELETE_MARKER_MSG = Marker(action=Marker.DELETEALL)
        self.DELETE_MARKER_ARRAY_MSG = MarkerArray(markers=[self.DELETE_MARKER_MSG])
        self.publishers = self.create_publishers()


    def create_publishers(self):
        pubs = dict()
        pubs["workspace"] = rospy.Publisher("/workspace", Marker, queue_size=1, latch=True)
        pubs["tsdf"] = rospy.Publisher("/tsdf_gen", PointCloud2, queue_size=1, latch=True)
        pubs["points"] = rospy.Publisher("/points", PointCloud2, queue_size=1, latch=True)
        pubs["quality"] = rospy.Publisher("/quality", PointCloud2, queue_size=1, latch=True)
        pubs["grasp"] = rospy.Publisher("/grasp", MarkerArray, queue_size=1, latch=True)
        pubs["grasps"] = rospy.Publisher("/grasps", MarkerArray, queue_size=1, latch=True)
        pubs["debug"] = rospy.Publisher("/debug", PointCloud2, queue_size=1, latch=True)
        return pubs

    def clear(self):
        self.publishers["workspace"].publish(self.DELETE_MARKER_MSG)
        self.publishers["tsdf"].publish(utils.to_cloud_msg(np.array([]), frame="task"))
        self.publishers["points"].publish(utils.to_cloud_msg(np.array([]), frame="task"))
        self.clear_quality()
        self.publishers["grasp"].publish(self.DELETE_MARKER_ARRAY_MSG)
        self.clear_grasps()
        self.publishers["debug"].publish(utils.to_cloud_msg(np.array([]), frame="task"))

    def clear_quality(self):
        self.publishers["quality"].publish(utils.to_cloud_msg(np.array([]), frame="task"))

    def clear_grasps(self):
        self.publishers["grasps"].publish(self.DELETE_MARKER_ARRAY_MSG)

    def draw_workspace(self, size=0.05):
        # scale to dimension the lines of the workspace visualization (scale.x is used to control the width of the line segments): default 0.05 m * 0.005
        scale = size * 0.005
        # Initialize with Identity Transformation
        rotation = scipy.spatial.transform.Rotation.from_quat([0.0, 0.0, 0.0, 1.0])
        translation = np.array([0.0, 0.0, 0.0])
        pose = rotation, translation
        scale = [scale, 0.0, 0.0]
        color = [0.5, 0.5, 0.5]
        msg = utils.create_marker_msg(marker_type=Marker.LINE_LIST, frame="voxel_grid_origin", pose=pose, scale=scale, color=color)
        msg.points = [utils.to_point_msg(point) for point in utils.workspace_lines(dst=0.3)]

        self.publishers["workspace"].publish(msg)
        # Set publish-rate to 10 Hz
        rospy.sleep(0.1)

    def draw_tsdf(self, vol, voxel_size=0.0075, threshold=0.01):
        # vol = vol[:20,:,:]
        msg = utils.create_vol_msg(vol, voxel_size, threshold)
        '''while not rospy.is_shutdown():
            self.publishers["tsdf"].publish(msg)
            rospy.sleep(0.1)'''
        self.publishers["tsdf"].publish(msg)

    def draw_quality(self, vol, voxel_size, threshold=0.01):
        msg = utils.create_vol_msg(vol, voxel_size, threshold)
        self.publishers["quality"].publish(msg)

    def draw_grasps(self, grasps, scores, finger_depth):
        markers = []
        for i, (grasp, score) in enumerate(zip(grasps, scores)):
            msg = utils.create_grasp_marker_msg(grasp, score, finger_depth)
            msg.id = i
            markers.append(msg)
        msg = MarkerArray(markers=markers)
        self.publishers["grasps"].publish(msg)