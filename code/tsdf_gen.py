######################################################################
# File: tsdf_gen.py
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
import argparse
import matplotlib
from mesh_to_sdf import sample_sdf_near_surface, get_surface_point_cloud, mesh_to_voxels
import trimesh
import skimage
import pyrender
import time
import numpy as np
import utils
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import scipy.spatial.transform
import std_msgs.msg, tf2_ros
import open3d as o3d


class TSDFVol(object):
    def __init__(self, object='Ketchup'):
        voxel_res = 40
        surface_point_method = 'scan'
        sign_method = 'normal'
        scan_count = 100
        scan_res = 400
        sample_point_count = 1e6
        return_grad = False


        self.obj_root = '/home/nico/tsdf_generation/data/hope_meshes_eval/' + object + '.obj'
        self.mesh = self.load_obj(obj_root=self.obj_root)

        self.voxels = self.voxelize(voxel_res, scan_count, sign_method, surface_point_method, return_grad, scan_res, sample_point_count)
        self.voxels = (self.voxels + 1) * 0.5


    def load_obj(self, obj_root):
        return trimesh.load(obj_root)

    def voxelize(self, voxel_res, scan_count, sign_method, surface_point_method, return_grad, scan_res, sample_point_count):
        return mesh_to_voxels(self.mesh, voxel_resolution=voxel_res, check_result=False, scan_count=scan_count,
                                     sign_method=sign_method, surface_point_method=surface_point_method, return_gradients=return_grad,
                                     scan_resolution=scan_res, sample_point_count=sample_point_count)



class Visualization_Rviz(object):

    def __init__(self):
        rospy.init_node('node_tsdf_generation', anonymous=False)
        self.DELETE_MARKER_MSG = Marker(action=Marker.DELETEALL)
        self.DELETE_MARKER_ARRAY_MSG = MarkerArray(markers=[self.DELETE_MARKER_MSG])
        self.publishers = self.create_publishers()
        self.clear()


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
        scale = size * 0.005
        # Initialize with Identity Transformation
        rotation = utils.Rotation.from_quat([0.0, 0.0, 0.0, 1.0])
        translation = np.array([0.0, 0.0, 0.0])
        print(rotation)
        scale = [scale, 0.0, 0.0]
        color = [0.5, 0.5, 0.5]
        msg = _create_marker_msg(Marker.LINE_LIST, "task", pose, scale, color)
        msg.points = [utils.to_point_msg(point) for point in workspace_lines(size)]
        self.publishers["workspace"].publish(msg)


def main(args):
    tsdf_vol = TSDFVol(args.object)
    print(tsdf_vol.voxels.shape, np.amax(tsdf_vol.voxels), np.amin(tsdf_vol.voxels), np.expand_dims(tsdf_vol.voxels, axis=0).shape)

    if args.viz == True:
        visualization = Visualization_Rviz()



if __name__ == '__main__':
    print('Starting TSDF-Generation at {}\n'.format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', type=str, default='Ketchup')
    parser.add_argument("--viz", action="store_true", default=True)
    args = parser.parse_args()
    main(args)




