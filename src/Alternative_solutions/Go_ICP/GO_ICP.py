#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 11:25:50 2021

@author: sebastianhedegaardhansen
"""
import numpy as np;
import open3d as o3d
import time 
import copy
import Go_ICP_function as icpf
from py_goicp import GoICP, POINT3D, ROTNODE, TRANSNODE



def loadPointCloud(filename):
    pcloud = np.loadtxt(filename, skiprows=1);
    plist = pcloud.tolist();
    p3dlist = [];
    for x,y,z in plist:
        pt = POINT3D(x,y,z);
        p3dlist.append(pt);
    return pcloud.shape[0], p3dlist;

def load_point_clouds_ply(file_name, normalize = True, remove_plane = False):
    pcd = o3d.io.read_point_cloud(file_name)  
    
    if remove_plane == True:
        pcd = icpf.demo_crop_geometry(pcd)
        
    #pcd, _ = pcd.remove_radius_outlier(nb_points=16, radius=0.5)
    
    pc_cloud = []
    number_points = len(np.asarray(pcd.points))
    p3dlist = []
    if normalize == True:
        cloud = np.asarray(pcd.points)
        cloud = normalize_function(cloud)
    else:
        cloud = np.asarray(pcd.points)
    
    for x,y,z in cloud:
        pc_cloud.append([x,y,z])
        pt = POINT3D(x,y,z)
        p3dlist.append(pt)
    return pcd, number_points, p3dlist, pc_cloud

def normalize_function(x):
    min_norm = x.min()
    max_norm = x.max()
    
    return 2*((x-min_norm)/(max_norm-min_norm))-1
    
    
    
goicp = GoICP()
rNode = ROTNODE()
tNode = TRANSNODE()
 
rNode.a = -3.1416
rNode.b = -3.1416
rNode.c = -3.1416
rNode.w = 6.2832
 
tNode.x = -0.5
tNode.y = -0.5
tNode.z = -0.5
tNode.w = 1.0

#threadshold mean Square Error


  
if(goicp.trimFraction < 0.0001):
    goicp.doTrim = False
    


target, Nm, a_points, pc_cloud_target = load_point_clouds_ply('cropped_multiway_registration_top_full_clean.ply', normalize = True, remove_plane = False)
#source2, Nc, c_points, pc_cloud_source2 = load_point_clouds_ply('multiway_registration_inner_full.ply', normalize = True)
source1, Nd, b_points, pc_cloud_source1 = load_point_clouds_ply('cropped_multiway_registration_inner_full_clean.ply', normalize = True, remove_plane = False)

num_points = (Nm+Nd)
trim = 0.2
goicp.trimFraction = trim 

mse = 0.001 * ((1-trim)*num_points)

goicp.MSEThresh = mse


pcd_source1 = o3d.geometry.PointCloud()
pcd_source1.points = o3d.utility.Vector3dVector(np.asarray(pc_cloud_source1))

#pcd_source2 = o3d.geometry.PointCloud()
#pcd_source2.points = o3d.utility.Vector3dVector(np.asarray(pc_cloud_source2))

pcd_target = o3d.geometry.PointCloud()
pcd_target.points = o3d.utility.Vector3dVector(np.asarray(pc_cloud_target))
# =============================================================================
# target, Nm, a_points = load_point_clouds_ply('go-icp_cython-master/tests/model_bunny.ply', normalize = False)
# source, Nd, b_points = load_point_clouds_ply('go-icp_cython-master/tests/data_bunny.ply', normalize = False)
# 
# =============================================================================

#o3d.visualization.draw_geometries([pcd_target, pcd_source1])


goicp.loadModelAndData(Nm, a_points, Nd, b_points)
# DT = Euclidean Distance Transform 
goicp.setDTSizeAndFactor(300, 2.0)
goicp.setInitNodeRot(rNode)
goicp.setInitNodeTrans(tNode)

start = time.time()

goicp.BuildDT()
goicp.Register()

end = time.time()
optR = np.array(goicp.optimalRotation())
optT = goicp.optimalTranslation()
optT.append(1.0)
optT = np.array(optT)

transforms = np.zeros((4,4))
transforms[:3, :3] = optR
transforms[:,3] = optT
pcd_source1.transform(transforms)


o3d.visualization.draw_geometries([pcd_source1, pcd_target])
print('TOTAL TIME : ', (end-start))

#Combine the two point clouds
pcd_combined = o3d.geometry.PointCloud()
pcd_combined = pcd_source1 + pcd_target
pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=0.005)
#o3d.visualization.draw_geometries([pcd_combined_down])
o3d.io.write_point_cloud("Top_inner.ply", pcd_combined_down)

#target, Nm, a_points, pc_cloud_target = load_point_clouds_ply('outer_top.ply', normalize = False)

# =============================================================================
# 
# goicp.loadModelAndData(Nm, a_points, Nc, c_points)
# # DT = Euclidean Distance Transform 
# goicp.setDTSizeAndFactor(300, 2.0)
# goicp.setInitNodeRot(rNode)
# goicp.setInitNodeTrans(tNode)
# 
# start = time.time()
# 
# goicp.BuildDT()
# goicp.Register()
# 
# end = time.time()
# optR = np.array(goicp.optimalRotation())
# optT = goicp.optimalTranslation()
# optT.append(1.0)
# optT = np.array(optT)
# 
# transforms = np.zeros((4,4))
# transforms[:3, :3] = optR
# transforms[:,3] = optT
# pcd_source2.transform(transforms)
# 
# 
# o3d.visualization.draw_geometries([pcd_source2, pcd_target])
# print('TOTAL TIME : ', (end-start))
# 
# =============================================================================

# =============================================================================
# print(goicp.optimalRotation()); # A python list of 3x3 is returned with the optimal rotation
# print(goicp.optimalTranslation());# A python list of 1x3 is returned with the optimal translation
# 
# 
# =============================================================================
