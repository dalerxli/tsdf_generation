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

from mesh_to_sdf import sample_sdf_near_surface, get_surface_point_cloud, mesh_to_voxels
import trimesh
import skimage
import pyrender
import time
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg, tf2_ros
import open3d as o3d


class TSDFVol(object):
    def __init__(self, object='Ketchup', viz=True):
        voxel_res = 40
        surface_point_method = 'scan'
        sign_method = 'normal'
        scan_count = 100
        scan_res = 400
        sample_point_count = 1e6
        return_grad = False


        self.obj_root = '/home/nleuze/generate_tsdf/data/hope_meshes_eval/' + object + '.obj'
        self.mesh = self.load_obj(obj_root=self.obj_root)

        self.voxels = self.voxelize(voxel_res, scan_count, sign_method, surface_point_method, return_grad, scan_res, sample_point_count)

        if viz == True:
            ff

    def load_obj(self, obj_root):
        return trimesh.load(obj_root)

    def voxelize(self, voxel_res, scan_count, sign_method, surface_point_method, return_grad, scan_res, sample_point_count):
        return mesh_to_voxels(self.mesh, voxel_resolution=voxel_res, check_result=False, scan_count=scan_count,
                                     sign_method=sign_method, surface_point_method=surface_point_method, return_gradients=return_grad,
                                     scan_resolution=scan_res, sample_point_count=sample_point_count)



'''voxels = mesh_to_voxels(mesh, 40, pad=False, surface_point_method='scan', check_result=True, scan_count=100)
voxels = np.asarray(voxels)
np.savez('voxels.npz', voxels)

voxels = (voxels + 1) * 0.5
np.expand_dims(voxels, axis=0)
print(voxels.shape)'''

class Visualization_Rviz(object):

    def __init__(self):
        xx = 1



if __name__ == '__main__':
    print('Starting TSDF-Generation at {}'.format(time.time()))




