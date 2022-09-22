######################################################################
# File: transformation_robot.py
# Project: Research Project (M.Sc.)
# Created Date: 18.09.2022
# Author: Nico Leuze,
#         Faculty of Electrical Engineering and Information Technology
#         University of Applied Science Munich (MUAS)
# -----
# Last Modified: 19.09.2022
# Modified By: Nico Leuze
# -----
# Copyright (c) 2022 MUAS
######################################################################
#!/usr/bin/python3
import roslib
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

class Transformation_Grasp(object):
    def __init__(self, vgn_grasp):
        self.listener_centroid_origin = tf.TransformListener()
        self.listener_centroid_origin.waitForTransform('/centroid_voxel_grid', '/voxel_grid_origin', rospy.Time(),
                                                  rospy.Duration(4.0))
        self.trafo_centroid_origin = self.listener_centroid_origin.lookupTransform('/centroid_voxel_grid', '/voxel_grid_origin',
                                                                         rospy.Time(0))
        # print(self.trafo_centroid_origin)
        self.pose_voxelorigin_grasp = vgn_grasp[0]
        self.pose_voxelorigin_grasp_t = self.pose_voxelorigin_grasp.pose.translation
        self.pose_voxelorigin_grasp_r = self.pose_voxelorigin_grasp.pose.rotation.as_quat()
        print('Transformation!! ', self.pose_voxelorigin_grasp_t, self.pose_voxelorigin_grasp_r)
        self.grasp_pose_transformation()

    def dope_pose_callback(self):
        # DOPE predicted Pose: Object Pose w.r.t. Camera:
        dev_bool = False
        if dev_bool == True:
            self.dope_pose_cam_object = rospy.wait_for_message("/dope/pose_sugar", PoseStamped)
            self.dope_pose_cam_object_t = [self.dope_pose_cam_object.pose.position.x, self.dope_pose_cam_object.pose.position.y,
                                           self.dope_pose_cam_object.pose.position.z]
            self.dope_pose_cam_object_r = [self.dope_pose_cam_object.pose.orientation.x, self.dope_pose_cam_object.pose.orientation.y,
                                           self.dope_pose_cam_object.pose.orientation.z, self.dope_pose_cam_object.pose.orientation.w]

            # Get the transformation between the base_link and the camera_color_optical_frame
            self.pose_base_cam_listener = tf.TransformListener()
            self.pose_base_cam_listener.waitForTransform("/base_link", "/camera_color_optical_frame_calibrated", rospy.Time(),
                                                          rospy.Duration(4.0))
            self.transformation_base_cam_t, self.transformation_base_cam_r = self.pose_base_cam_listener.lookupTransform("/base_link",
                                                                                        "/camera_color_optical_frame_calibrated",
                                                                                        rospy.Time(0))

        else:
            self.dope_pose_cam_object_t = [1.0, 0.0, -1.0]
            self.dope_pose_cam_object_r = [0.0, 0.0, 0.0, 1.0]
            self.transformation_base_cam_t = [1.0, 0.0, 0.0]
            self.transformation_base_cam_r = [0.0, 0.0, 0.0, 1.0]

        # Generate Euler Representation:
        rx, ry, rz = tf.transformations.euler_from_quaternion(self.dope_pose_cam_object_r)
        euler_trafo_cam_object = tf.transformations.euler_matrix(rx, ry, rz, 'sxyz')
        euler_trafo_cam_object[0:3, 3] = self.dope_pose_cam_object_t

        rx, ry, rz = tf.transformations.euler_from_quaternion(self.transformation_base_cam_r)
        euler_trafo_base_cam = tf.transformations.euler_matrix(rx, ry, rz, 'sxyz')
        euler_trafo_base_cam[0:3, 3] = self.transformation_base_cam_t

    def grasp_pose_transformation(self):
        # Generate Euler Representation for Centroid to Origin of VoxelGrid:
        rx_co, ry_co, rz_co = tf.transformations.euler_from_quaternion(self.trafo_centroid_origin[1])
        self.euler_trafo_centroid_origin = tf.transformations.euler_matrix(rx_co, ry_co, rz_co)
        self.euler_trafo_centroid_origin[0:3, 3] = self.trafo_centroid_origin[0]

        # Generate Euler Representation for Origin of VoxelGrid to Grasp Prediction:
        rx_og, ry_og, rz_og = tf.transformations.euler_from_quaternion(self.pose_voxelorigin_grasp_r)
        self.euler_trafo_origin_grasp = tf.transformations.euler_matrix(rx_og, ry_og, rz_og)
        self.euler_trafo_origin_grasp[0:3, 3] = self.pose_voxelorigin_grasp_t

        # Matrix Composition to obtain transformation between Centroid of VoxelGrid to Grasp Prediction:
        self.transformation_centroid_grasp = np.matmul(self.euler_trafo_centroid_origin, self.euler_trafo_origin_grasp)

        # Call Function 'dope_pose_callback' to obtain the (DOPE) predicted Pose of the Object Centroid (== VoxelGridCentroid) wrt. to the robot_base
        self.dope_pose_callback()

        # Matrix Composition to obtain transformation between Robot_Base and Grasp Prediction:



def main():
    listener_origin_centroid = tf.TransformListener()
    listener_origin_centroid.waitForTransform('/voxel_grid_origin', 'centroid_voxel_grid', rospy.Time(), rospy.Duration(4.0))
    listener_origin_centroid.waitForTransform('/robot_base', 'centroid_voxel_grid', rospy.Time(), rospy.Duration(9.0))
    trafo_origin_centroid = listener_origin_centroid.lookupTransform('/voxel_grid_origin', 'centroid_voxel_grid', rospy.Time(0))
    print(trafo_origin_centroid)
    rx, ry, rz = tf.transformations.euler_from_quaternion(trafo_origin_centroid[1])
    euler_trafo_origin_centroid = tf.transformations.euler_matrix(rx, ry, rz, 'sxyz')
    euler_trafo_origin_centroid[0:3, 3] = trafo_origin_centroid[0]
    print('Transformation Origin2Centroid\n', trafo_origin_centroid, '\n', euler_trafo_origin_centroid, '\n\n')

    trafo_base_origin = listener_origin_centroid.lookupTransform('/robot_base', 'voxel_grid_origin', rospy.Time(0))
    rx, ry, rz = tf.transformations.euler_from_quaternion(trafo_base_origin[1])
    euler_trafo_base_origin = tf.transformations.euler_matrix(rx, ry, rz, 'sxyz')
    euler_trafo_base_origin[0:3, 3] = trafo_base_origin[0]
    print('Transformation Base2Origin\n', trafo_base_origin, '\n', euler_trafo_base_origin, '\n\n')

    trafo_base_centroid = listener_origin_centroid.lookupTransform('/robot_base', 'centroid_voxel_grid', rospy.Time(0))
    rx, ry, rz = tf.transformations.euler_from_quaternion(trafo_base_centroid[1])
    euler_trafo_base_centroid = tf.transformations.euler_matrix(rx, ry, rz, 'sxyz')
    euler_trafo_base_centroid[0:3, 3] = trafo_base_centroid[0]
    print('Transformation Base2Centroid\n', trafo_base_centroid, '\n', euler_trafo_base_centroid, '\n\n')

    trafo_centroid_origin = listener_origin_centroid.lookupTransform('/centroid_voxel_grid', 'voxel_grid_origin', rospy.Time(0))
    rx, ry, rz = tf.transformations.euler_from_quaternion(trafo_centroid_origin[1])
    euler_trafo_centroid_origin = tf.transformations.euler_matrix(rx, ry, rz, 'sxyz')
    euler_trafo_centroid_origin[0:3, 3] = trafo_centroid_origin[0]
    print('!!! Transformation Centroid2Origin\n', trafo_centroid_origin, '\n', euler_trafo_centroid_origin, '\n\n')


    # Calculation Exmaple with predicted Grasp Pose from VGN:
    trafo_origin_grasp_translation = [0.2175, 0.18, 0.2475]             # means: x = 29 voxels, y = 24 voxels, z = 33 voxels
    trafo_origin_grasp_rotation = [-0.8312564436114758, -0.24226918653031623, 0.48108249577988293, -0.13739722874402915]
    rx, ry, rz = tf.transformations.euler_from_quaternion(trafo_origin_grasp_rotation)
    euler_trafo_origin_grasp = tf.transformations.euler_matrix(rx, ry, rz, 'sxyz')
    euler_trafo_origin_grasp[0:3, 3] = trafo_origin_grasp_translation
    print('Transformation Origin2Grasp\n', trafo_origin_grasp_translation, trafo_origin_grasp_rotation, '\n', euler_trafo_origin_grasp, '\n\n')

    # Checking Method:
    euler_trafo_centroid_grasp = np.matmul(euler_trafo_centroid_origin, euler_trafo_origin_grasp)
    print('Transformation Centroid to Grasp: \n', euler_trafo_centroid_grasp, '\n\n')
    # With the given Setup we get the following translational displacements: [0.0675, 0.03, 0.0975] >> means: x = 9 voxels, y = 4 voxels, z = 13

    # Calculating resulting Pose Robot_Base to predicted Grasp:
    euler_trafo_robot_base_to_origin = np.matmul(euler_trafo_base_centroid, euler_trafo_centroid_origin)
    euler_trafo_robot_base_to_grasp = np.matmul(euler_trafo_robot_base_to_origin, euler_trafo_origin_grasp)
    print('Transformation Robot Base to Grasp: \n', euler_trafo_robot_base_to_grasp)

if __name__=='__main__':
    rospy.init_node('trafo_dev_node')
    try:
        main()
    except rospy.ROSInterruptException:
        pass