#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 14:53:51 2021

@author: sebastianhedegaardhansen
"""
import open3d as o3d
import numpy as np
import copy
import matplotlib.pyplot as plt
import sklearn as sk
from sklearn import cluster, datasets, mixture
voxel_size = 0.1

#Function for loading point cloud 

def load_point_clouds(images, data_path, voxel_size=0.1):
    pcds = []
    for i, img in enumerate(images):
        pcd = o3d.io.read_point_cloud(f"{data_path}/points{img}.ply")  
        pcds.append(pcd)
    return pcds




#Stitch two clouds with course and then fine 
def pairwise_registration(source, target, 
                          max_correspondence_distance_coarse, max_correspondence_distance_fine,
                          max_iter_corse, max_iter_fine):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = max_iter_corse))
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = max_iter_fine))
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp

#Stitch all the clouds and return pose graph 
def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine, max_iter_corse, max_iter_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id],
                max_correspondence_distance_coarse, max_correspondence_distance_fine,
                max_iter_corse, max_iter_fine)
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

#Radius outlier removal. 
def outlier_removed(pointCloud):
    inliner, outlier_index_source = pointCloud.remove_radius_outlier(
                                              nb_points=16,
                                              radius=0.5)
    return inliner


#DBSCAN
def clusterDBSCAN(X, min_size = 1):
    db = cluster.DBSCAN(eps=0.3, min_samples = min_size).fit(X)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    return db

#Find clusters and remove them 
def removeOutliers(cloud, threshold = 1000):
    db = clusterDBSCAN(np.asarray(cloud.points))
    #count cluster
    number_of_cl = max(db.labels_)
    unique, c = np.unique(db.labels_, return_counts=True)
    counts = dict(zip(unique,c))
    final_cl = [0]*len(db.labels_)
    
    points_old = np.asarray(cloud.points)
    points_new = []
    for i in range(len(db.labels_)):
        if counts[db.labels_[i]] > threshold:
            #final_cl[i] = 1
            points_new.append(points_old[i])
        #else:
            #final_cl[i] = 0
        #print(i,db.labels_[i],counts[db.labels_[i]],final_cl[i])
            
    #points = np.asarray(cloud.points)[final_cl]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_new)
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    return pcd










