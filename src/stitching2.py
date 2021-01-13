import open3d as o3d
import numpy as np
import copy
from param import *
from help_functions import *
import pickle as pkl
import os
import re
import time
from clustering import *




def global_and_icp_registration(source, target, title = "temp"):
    source_down, target_down, source_fpfh, target_fpfh, processed_source, processed_target, trans_init= prepare_dataset(voxel_size, source, target)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size, trans_init)
    print("result_ransac")

    print("inlier_rmse = ",result_ransac.inlier_rmse)
    print("fitness = ",result_ransac.fitness)

    # draw_registration_result(source_down, target_down, result_ransac.transformation)

    result_icp = refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac)

    # draw_registration_result(source, target, result_icp.transformation)

    result = save_transformation(source, target, result_icp.transformation, title,  True)
    # result = save_registration_result(source, target, result_icp.transformation, title)

    return result, result_icp.transformation


def print_point_cloud(filename, title):
    source = o3d.io.read_point_cloud(filename)
    o3d.visualization.draw_geometries([source], width=1000, height=800, window_name='Open3D-'+str(title))


t0= time.clock()

external_path = "../data/plyfolder2/external/"
# internal_path = "../data/plyfolder2/internal/"
upper_path = "../data/plyfolder2/upper/"


external = os.listdir(external_path)
# internal = os.listdir(internal_path)
upper    = os.listdir(upper_path)

# ========================== Stitching External Side =======================================

print("external")
source = o3d.io.read_point_cloud(external_path + external[0])
for i in range(len(external) - 1):
    target = o3d.io.read_point_cloud(external_path + external[i+1])
    source, transformation = global_and_icp_registration(source, target)


processed_source, outlier_index = source.remove_radius_outlier( nb_points=25, radius=0.5)

# save pointcloud
filename = "../stiched/external.pcd"
o3d.io.write_point_cloud(filename, processed_source)

# print_point_cloud(filename, "external")


# ========================== Stitching Internal Side =======================================

# print("internal")
# source = o3d.io.read_point_cloud(internal_path + internal[0])
# for i in range(len(internal) - 1):
#     target = o3d.io.read_point_cloud(internal_path + internal[i+1])
#     source, transformation = global_and_icp_registration(source, target)


# processed_source, outlier_index = source.remove_radius_outlier( nb_points=25, radius=0.5)

# # save pointcloud
# filename = "../stiched/internal.pcd"
# o3d.io.write_point_cloud(filename, processed_source)

# # print_point_cloud(filename, "internal")


# ========================== Stitching Upper Side =======================================


print("upper")
source = o3d.io.read_point_cloud(upper_path + upper[0])
for i in range(len(upper) - 1):
    target = o3d.io.read_point_cloud(upper_path + upper[i+1])
    source, transformation = global_and_icp_registration(source, target)


processed_source, outlier_index = source.remove_radius_outlier( nb_points=25, radius=0.5)

# save pointcloud
filename = "../stiched/upper.pcd"
o3d.io.write_point_cloud(filename, processed_source)

# print_point_cloud(filename, "upper")


# ========================== Stitching the 3 Sides =======================================

external_pcd = o3d.io.read_point_cloud("../stiched/external.pcd")
# internal_pcd = o3d.io.read_point_cloud("../stiched/internal.pcd")
upper_pcd    = o3d.io.read_point_cloud("../stiched/upper.pcd")

external_upper, transformation = global_and_icp_registration(external_pcd, upper_pcd)
# final, transformation = global_and_icp_registration(external_upper, internal_pcd)

# processed_final, outlier_index = final.remove_radius_outlier( nb_points=50, radius=0.5)
processed_ext_upper, outlier_index = external_upper.remove_radius_outlier( nb_points=50, radius=0.5)

# save pointcloud
filename = "../stiched/final2.pcd"
# o3d.io.write_point_cloud(filename, processed_final)
o3d.io.write_point_cloud(filename, processed_ext_upper)


cluster_pcd = clustering(processed_ext_upper)

t1 = time.clock() - t0
print("Time elapsed: ", t1, "seconds") # CPU seconds elapsed (floating point)

custom_draw_geometry(cluster_pcd)

filename = "../stiched/final2_clustered.pcd"
o3d.io.write_point_cloud(filename, cluster_pcd)

