######################################################################
# File: vgn_utils.py
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

import time

import numpy as np
from scipy import ndimage
import torch
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
# from vgn import vis
# from vgn.grasp import *
from utils import Rotation, Transform, Grasp
from vgn_network import load_network
import visualization

class VGN(object):
    def __init__(self, model_path=None, rviz=True):
        if model_path == None:
            model_path = Path('/home/nleuze/vgn_grasp/src/vgn/data/models/vgn_conv.pth')
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.net = load_network(model_path, self.device)
        self.rviz = rviz

    def __call__(self, tsdf_vol, voxel_size):
        # Get Shape to (1,40,40,40):
        tsdf_vol = np.expand_dims(tsdf_vol, axis=0)
        visualization_ = visualization.Visualization_Rviz()

        tic = time.time()
        qual_vol, rot_vol, width_vol = predict(tsdf_vol, self.net, self.device)
        visualization_.draw_quality(qual_vol, voxel_size, threshold=0.1)
        qual_vol, rot_vol, width_vol = process(tsdf_vol, qual_vol, rot_vol, width_vol)
        grasps, scores = select(qual_vol.copy(), rot_vol, width_vol)
        toc = time.time() - tic

        grasps, scores = np.asarray(grasps), np.asarray(scores)

        print('VGN found {} plausible grasp candidates!'.format(len(grasps)))

        if len(grasps) > 0:
            p = np.random.permutation(len(grasps))
            grasps = [from_voxel_coordinates(g, voxel_size) for g in grasps[p]]
            scores = scores[p]

        # visualization_ = visualization.Visualization_Rviz()
        visualization_.draw_quality(qual_vol, voxel_size, threshold=0.01)

        return grasps, scores, toc


def predict(tsdf_vol, net, device):
    assert tsdf_vol.shape == (1, 40, 40, 40)

    # move input to the GPU
    tsdf_vol = torch.from_numpy(tsdf_vol).unsqueeze(0).to(device)
    print('\n\n Input NET: \n', tsdf_vol, '\n\n')#Shape Input: ', tsdf_vol.shape, '\n\n')
    # forward pass
    with torch.no_grad():
        qual_vol, rot_vol, width_vol = net(tsdf_vol)

    # move output back to the CPU
    qual_vol = qual_vol.cpu().squeeze().numpy()
    rot_vol = rot_vol.cpu().squeeze().numpy()
    width_vol = width_vol.cpu().squeeze().numpy()
    return qual_vol, rot_vol, width_vol


def process(
    tsdf_vol,
    qual_vol,
    rot_vol,
    width_vol,
    gaussian_filter_sigma=1.0,
    min_width=1.33,
    max_width=20,
):
    tsdf_vol = tsdf_vol.squeeze()
    # qual_64 = np.float64(qual_vol)
    # np.save('/home/nleuze/qual_vol_notprocessed.npy', qual_64)

    # smooth quality volume with a Gaussian
    qual_vol = ndimage.gaussian_filter(
        qual_vol, sigma=gaussian_filter_sigma, mode="nearest"
    )

    # mask out voxels too far away from the surface
    outside_voxels = tsdf_vol > 0.5
    inside_voxels = np.logical_and(1e-3 < tsdf_vol, tsdf_vol < 0.5)
    valid_voxels = ndimage.morphology.binary_dilation(
        outside_voxels, iterations=2, mask=np.logical_not(inside_voxels)
    )
    qual_vol[valid_voxels == False] = 0.0

    # reject voxels with predicted widths that are too small or too large
    qual_vol[np.logical_or(width_vol < min_width, width_vol > max_width)] = 0.0

    return qual_vol, rot_vol, width_vol


def select(qual_vol, rot_vol, width_vol, threshold=0.50, max_filter_size=3):
    # threshold on grasp quality
    # grasps, scores = select(qual_vol.copy(), rot_vol, width_vol)
    # fig = plt.figure()
    qual_vol_hist = np.reshape(qual_vol, (1, 64000))
    '''plt.hist(qual_vol_hist)
    plt.title('Distribution of Quality Scores')
    plt.xlabel('Quality Score')
    plt.ylabel('Occurrence')'''
    # plt.show()

    qual_vol[qual_vol < threshold] = 0.0
    qual_64 = np.float64(qual_vol)
    np.save('/home/nleuze/qual_vol_k3.npy', qual_64)


    # non maximum suppression
    max_vol = ndimage.maximum_filter(qual_vol, size=max_filter_size)
    # max_64 = np.float64(max_vol)
    # np.save('/home/nleuze/max_vol_k3.npy', max_64)

    # Visualization of the Maximumfilter Volume and Quality Volume:
    # max_img = max_vol[20, :, :]
    # qual_img = qual_vol[20, :, :]
    nms_img = np.where(qual_vol == max_vol, qual_vol, 0.0)
    nms_64 = np.float64(nms_img)
    # np.save('/home/nleuze/nms_vol_k3.npy', nms_64)
    print()

    qual_vol = np.where(qual_vol == max_vol, qual_vol, 0.0)

    # Test for Transformation:
    qual_vol[0, 0, 0] = 0.51
    # qual_vol[1, 1, 1] = 0.97
    # qual_vol[2, 2, 2] = 0.97
    mask = np.where(qual_vol, 1.0, 0.0)


    # New Method:
    '''def select_local_maxima(voxel_size, out, threshold=0.9, max_filter_size=3.0):
    max = ndimage.maximum_filter(out.qual, size=max_filter_size)
    index_list = np.argwhere(np.logical_and(out.qual == max, out.qual > threshold))
    grasps, qualities = [], []
    for index in index_list:
        grasp, quality = select_at(out, index)
        grasps.append(grasp)
        qualities.append(quality)
    grasps = np.array([from_voxel_coordinates(voxel_size, g) for g in grasps])
    qualities = np.asarray(qualities)
    return grasps, qualities'''


    # construct grasps
    grasps, scores = [], []
    for index in np.argwhere(mask):
        grasp, score = select_index(qual_vol, rot_vol, width_vol, index)
        grasps.append(grasp)
        scores.append(score)

    return grasps, scores


def select_index(qual_vol, rot_vol, width_vol, index):
    i, j, k = index
    print(index)
    score = qual_vol[i, j, k]
    ori = Rotation.from_quat(rot_vol[:, i, j, k])
    pos = np.array([i, j, k], dtype=np.float64)
    width = width_vol[i, j, k]
    return Grasp(Transform(ori, pos), width), score

def from_voxel_coordinates(grasp, voxel_size):
    pose = grasp.pose
    pose.translation *= voxel_size
    width = grasp.width * voxel_size
    return Grasp(pose, width)

def explode(data):
    size = np.array(data.shape)*2
    data_e = np.zeros(size - 1, dtype=data.dtype)
    data_e[::2, ::2, ::2] = data
    return data_e
