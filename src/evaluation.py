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
# from stitching import global_and_icp_registration



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


dental = stl_to_pcd()
dental.paint_uniform_color([0.2, 0.7, 0.2])

transformation = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


center= np.asarray([0.0, 0.0, 1.0])

final = o3d.io.read_point_cloud("../stiched/final_clustered.pcd")

# draw_registration_result(dental, final, transformation)

scale_factor = 1000
trans_scale = np.asarray([[scale_factor, 0.0, 0.0, 0.0],
                            [0.0, scale_factor, 0.0, 0.0],
                            [0.0, 0.0, scale_factor, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])

temp_pcd = copy.deepcopy(dental)
temp_pcd.transform(trans_scale)

draw_registration_result(final, temp_pcd, transformation)

output, trans = global_and_icp_registration(final, temp_pcd, title = "eval")

draw_registration_result(final, temp_pcd, trans)

filename = "../stiched/evaluation.pcd"
o3d.io.write_point_cloud(filename, output)

