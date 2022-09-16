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
import skimage
import trimesh
import skimage
import pyrender
import time
import numpy as np
import utils
import rospy
from vgn_utils import VGN
import visualization



class TSDFVol(object):
    def __init__(self, object='Ketchup_cm'):
        voxel_res = 40
        workspace = 0.3                     # Unit [m]
        surface_point_method = 'scan'
        sign_method = 'normal'
        scan_count = 100
        scan_res = 400
        sample_point_count = 1e6
        return_grad = False
        # Truncation Distance adjustable: default = 4 * VoxelSize
        self.voxel_size = workspace / voxel_res
        self.trunc_dist = 4 * self.voxel_size


        self.obj_root = '/home/nleuze/tsdf_generation/data/thesis_test_objects/' + object + '.obj'
        self.mesh = self.load_obj(obj_root=self.obj_root)
        bbox = self.mesh.bounding_box.bounds
        centroid_bbox = (bbox[0] + bbox[1]) / 2
        # scale = (bbox[1] - bbox[0]).max()

        self.voxels = self.voxelize(voxel_res, scan_count, sign_method, surface_point_method, return_grad, scan_res, sample_point_count)

        # Reconstruct Object Mesh using Marching Cubes:
        # vertices, faces, normals, _ = skimage.measure.marching_cubes(self.voxels, level=0)
        # reconstructed_mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=normals)
        # reconstructed_mesh.show()
        
        self.truncate_sdf(self.trunc_dist)


    def load_obj(self, obj_root):
        return trimesh.load(obj_root)

    def voxelize(self, voxel_res, scan_count, sign_method, surface_point_method, return_grad, scan_res, sample_point_count):
        return mesh_to_voxels(self.mesh, voxel_resolution=voxel_res, check_result=False, scan_count=scan_count,
                                     sign_method=sign_method, surface_point_method=surface_point_method, return_gradients=return_grad,
                                     scan_resolution=scan_res, sample_point_count=sample_point_count)

    def truncate_sdf(self, trunc_dist):

        self.voxels[self.voxels > trunc_dist] = trunc_dist
        self.voxels[self.voxels < -trunc_dist] = -trunc_dist
        self.voxels /= trunc_dist
        # Tresholding to remove / (set to 0 > no visualization) / voxels to far away from the objects' surface
        self.voxels[self.voxels > 0.98] = -1.
        self.voxels[self.voxels < -0.98] = -1.

        self.voxels = (self.voxels + 1) * 0.5



def main(args):
    visualization_ = None
    tsdf_vol = TSDFVol(args.object)
    print(tsdf_vol.voxels.shape, np.amax(tsdf_vol.voxels), np.amin(tsdf_vol.voxels), np.expand_dims(tsdf_vol.voxels, axis=0).shape)
    print(tsdf_vol.voxel_size)

    if args.viz == True:
        visualization_ = visualization.Visualization_Rviz()
        visualization_.clear()
        visualization_.draw_workspace()
        visualization_.draw_tsdf(tsdf_vol.voxels)

    # Initiate Grasping Prediction based on VGN network
    grasp_predictor = VGN()
    grasps, scores, timing = grasp_predictor(tsdf_vol.voxels, tsdf_vol.voxel_size)

    # Draw Grasps if not empty:
    if len(grasps) != 0 and visualization_ != None:
        visualization_.draw_grasps(grasps=grasps, scores=scores, finger_depth=0.05)
    else:
        print('No sensible gripping configurations could be found. ')

    print(type(grasps))
    print()





if __name__ == '__main__':
    print('Starting TSDF-Generation at {}\n'.format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', type=str, default='Ketchup_cm_y2')
    parser.add_argument("--viz", action="store_true", default=True)
    args = parser.parse_args()
    main(args)




