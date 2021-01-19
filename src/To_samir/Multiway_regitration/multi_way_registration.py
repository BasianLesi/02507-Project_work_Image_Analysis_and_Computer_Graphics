#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 14:32:45 2021

@author: sebastianhedegaardhansen
"""
#Based on http://www.open3d.org/docs/latest/tutorial/Advanced/multiway_registration.html
#The code only use ICP for rough and fine tunning therefore the initial transformation need 
#to be good. 
#The side it workes great but between the inner, top, outer it lacks and therefore a 
#global registration is advised for inintial transformation 

import open3d as o3d
import numpy as np
import copy
import matplotlib.pyplot as plt
import Function_multi_way as fm
import global_registration_functions as grf
import itertools

#Data locatioon 
inner_img = ['0', '01', '02', '03', '04', '05', '06', '07', '08', '09']
outer_img = ['19', '18', '17', '16', '15', '14', '13', '12', '11', '10']#, '13', '14', '15', '16', '17', '18', '19']
#outer_img = outer_img.reverse()
top_img = ['20', '21', '22', '23', '24', '25', '26', '27', '28', '29']
data_path ="data/plyfolder"
voxel_size = 0.1

#Load the point clouds 
inner = fm.load_point_clouds(inner_img, data_path, voxel_size) 
outer = fm.load_point_clouds(outer_img, data_path, voxel_size) 
top = fm.load_point_clouds(top_img, data_path, voxel_size) 

areas = [inner, outer, top]  
pointcloud_areas = []
Area_name = ['_inner', '_outer', '_top']

#Loop over the 3 sides 
for area_name_index, area in enumerate(areas):
    
    #remove outliers
    for i in range(len(area)):
        area[i] = fm.removeOutliers(area[i])
    
       
    #collected = itertools.chain(inner, outer, top)
    #o3d.visualization.draw_geometries(inner)
    print("Recompute the normal of the downsampled point cloud")
    for point_id in range(len(area)): 
        area[point_id].estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=60))
    
    
    #This defines the jumps in each iterration and and the number of iterations 
    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 5
    max_correspondence_distance_fine = voxel_size * 0.5
    max_iter_corse = 30
    max_iter_fine = 30
    
    #The code works as follows: 
    #First a corce icp is made to get rough alignment 
    #Second the fine registration is performed. 
    #The Transformations between each point cloud is saved in a pose graph where each 
    #node is a point cloud and the edges strore the transformation relation between them. 
    
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = fm.full_registration(area,
                                       max_correspondence_distance_coarse,
                                       max_correspondence_distance_fine,
                                       max_iter_corse, max_iter_fine)
    
    
    #The graph is optimized 
    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph, o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(), option)
    
    
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(area)):
        area[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += area[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    o3d.io.write_point_cloud("multiway_registration"+Area_name[area_name_index]+"_full.ply", pcd_combined_down)
    o3d.visualization.draw_geometries([pcd_combined_down])
    
    #Add to combined pointcloud 
    pointcloud_areas.append(pcd_combined_down)
    
#Code used to test if it was possible to stich every side together: 

# =============================================================================
# o3d.visualization.draw_geometries(pointcloud_areas)
# 
# source_down, target_down, source_fpfh, target_fpfh = grf.prepare_dataset(
#     pointcloud_areas[0], pointcloud_areas[2], voxel_size)
# 
# results = grf.execute_global_registration(source_down, target_down, source_fpfh,
#                                 target_fpfh, voxel_size)
# 
# pointcloud_areas[0].transform(results.transformation)
# o3d.visualization.draw_geometries([pointcloud_areas[0], pointcloud_areas[2]])
#     
# =============================================================================

#combie 3 areas 
# =============================================================================
# init_trans = np.identity(4)
# 
# for cloud in range(len(pointcloud_areas)):
#     pointcloud_areas[cloud].transform(init_trans)
# 
# =============================================================================
#o3d.visualization.draw_geometries(pointcloud_areas)
# =============================================================================
# print("Recompute the normal of the downsampled point cloud")
# for normal_id in range(len(pointcloud_areas)): 
#     pointcloud_areas[normal_id].estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.6, max_nn=100))
# 
#     
# print("Full registration ...")
# max_correspondence_distance_coarse = voxel_size * 80
# max_correspondence_distance_fine = voxel_size * 8
# max_iter_corse = 5
# max_iter_fine = 10
#     
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     pose_graph_collected = fm.full_registration(pointcloud_areas,
#                                    max_correspondence_distance_coarse,
#                                    max_correspondence_distance_fine,
#                                    max_iter_corse, max_iter_fine)
# 
# 
# 
# print("Optimizing PoseGraph ...")
# option = o3d.pipelines.registration.GlobalOptimizationOption(
#     max_correspondence_distance=max_correspondence_distance_fine,
#     edge_prune_threshold=0.35,
#     reference_node=0)
# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#     o3d.pipelines.registration.global_optimization(
#         pose_graph_collected, o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
#         o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(), option)
#     
# 
# 
# 
# print("Transform points and display")
# for point_id in range(len(pointcloud_areas)):
#     print(pose_graph_collected.nodes[point_id].pose)
#     pointcloud_areas[point_id].transform(pose_graph_collected.nodes[point_id].pose)
# o3d.visualization.draw_geometries(pointcloud_areas)
#      
#     
# 
# =============================================================================













