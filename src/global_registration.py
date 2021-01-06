import open3d as o3d
import numpy as np
import copy
from param import *
from help_functions import *
import pickle as pkl



source, target, source_down, target_down, source_fpfh, target_fpfh, processed_source, processed_target, trans_init, p_source_down, p_target_down, p_source_fpfh,  p_target_fpfh = prepare_dataset(voxel_size)


result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print("result_ransac")
draw_registration_result(source_down, target_down, result_ransac.transformation)


result_icp = refine_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size, result_ransac)

draw_registration_result(source_down, target_down, result_icp.transformation)

# result_ransac2 = execute_global_registration(p_source_down, p_target_down,
#                                             p_source_fpfh, p_target_fpfh,
#                                             voxel_size)

# draw_registration_result(p_source_down, p_target_down, result_ransac2.transformation)




# result_icp = refine_registration(p_source_down, p_target_down,
#                                             p_source_fpfh, p_target_fpfh,
#                                             voxel_size, result_ransac2)

# draw_registration_result(p_source_down, p_target_down, result_icp.transformation)




# # save transformation
# with open('0601-transformation_result_icp2.pkl','wb') as f:
#     pkl.dump(result_icp.transformation, f)

# #test load
# with open('0601-transformation_result_icp2.pkl','rb') as f:
#     x = pkl.load(f)
#     print(x)


# # save combined pointcloud as pcd file
# save_registration_result(source, target, x,'0601-stitch_0-19-20.pcd' )




# print("Apply point-to-point ICP")
# reg_p2p = o3d.pipelines.registration.registration_icp(
#     source, target, threshold, trans_init,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint())
# print(reg_p2p)
# print("Transformation is:")
# print(reg_p2p.transformation)
# draw_registration_result(source_down, target_down, reg_p2p.transformation)

# reg_p2p = o3d.pipelines.registration.registration_icp(
#     source_down, target_down, threshold, trans_init,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
#     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000000, relative_fitness = 0.1))
# print(reg_p2p)
# print("Transformation is:")
# print(reg_p2p.transformation)
# draw_registration_result(source_down, target_down, reg_p2p.transformation)


# print("Apply point-to-plane ICP")

# radius_normal = voxel_size * 2
   
# processed_source.estimate_normals(
#     o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=3000))
# processed_target.estimate_normals(
#     o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=3000))

# reg_p2l = o3d.pipelines.registration.registration_icp(
#     source, target, threshold, trans_init,
#     o3d.pipelines.registration.TransformationEstimationPointToPlane())
# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)



# draw_registration_result(processed_source, processed_target, reg_p2l.transformation)




