import open3d as o3d
import numpy as np
import copy
from param import *
from help_functions import *
import pickle as pkl


# source = o3d.io.read_point_cloud(source_path)
# target = o3d.io.read_point_cloud(target_path)


# def global_registration(source, target):
#     source_down, target_down, source_fpfh, target_fpfh, processed_source, processed_target, trans_init= prepare_dataset(voxel_size, source, target)

#     result_ransac = execute_global_registration(source_down, target_down,
#                                                 source_fpfh, target_fpfh,
#                                                 voxel_size, trans_init)
#     print("result_ransac")

#     print("inlier_rmse = ",result_ransac.inlier_rmse)
#     print("fitness = ",result_ransac.fitness)

#     draw_registration_result(source_down, target_down, result_ransac.transformation)

#     result_icp = refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac)

#     draw_registration_result(source, target, result_icp.transformation)

#     result = save_transformation(source, target, result_icp.transformation, False)

#     return result, result_icp.transformation









