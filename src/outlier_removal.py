import open3d as o3d
import numpy as np
from help_functions import *
from param import *
from matplotlib import pyplot as plt

# ===========Not Working yet======================

# source_path = "../stiched/stitch_99_98_97.pcd"
source_path = "../stiched/external_clean.pcd"
# source_path = "../stiched/tempclean external.pkl"
# source_path = "../data/plyfolder/points0.ply"

source = o3d.io.read_point_cloud(source_path)

print(source)
print(np.asarray(source.points))
o3d.visualization.draw_geometries([source])

# processed_source, outlier_index = source.remove_radius_outlier(
#                                               nb_points=20,
#                                               radius=0.5)

# o3d.visualization.draw_geometries([processed_source])


# print("Load a ply point cloud, print it, and render it")
# pcd = source
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# print("Downsample the point cloud with a voxel of 0.02")
# voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
# o3d.visualization.draw_geometries([voxel_down_pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# print("Every 5th points are selected")
# uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
# o3d.visualization.draw_geometries([uni_down_pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])
                        
# def display_inlier_outlier(cloud, ind):
#     inlier_cloud = cloud.select_by_index(ind)
#     outlier_cloud = cloud.select_by_index(ind, invert=True)

#     print("Showing outliers (red) and inliers (gray): ")
#     outlier_cloud.paint_uniform_color([1, 0, 0])
#     inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
#     o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
#                                       zoom=0.3412,
#                                       front=[0.4257, -0.2125, -0.8795],
#                                       lookat=[2.6172, 2.0475, 1.532],
#                                       up=[-0.0694, -0.9768, 0.2024])


# print("Statistical oulier removal")
# cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30,
#                                                     std_ratio=2.0)
# display_inlier_outlier(voxel_down_pcd, ind)



# pcd = source
# o3d.visualization.draw_geometries([pcd])
# alpha = 0.03
# print(f"alpha={alpha:.3f}")
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
# mesh.compute_vertex_normals()
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)


# tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
# for alpha in np.logspace(np.log10(0.5), np.log10(0.01), num=4):
#     print(f"alpha={alpha:.3f}")
#     mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
#         pcd, alpha, tetra_mesh, pt_map)
#     mesh.compute_vertex_normals()
#     o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)


# pcd = o3d.io.read_point_cloud(source_path)

# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     labels = np.array(
#         pcd.cluster_dbscan(eps=0.2, min_points=10, print_progress=True))

# max_label = labels.max()
# print(f"point cloud has {max_label + 1} clusters")
# colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.455,
#                                   front=[-0.4999, -0.1659, -0.8499],
#                                   lookat=[2.1813, 2.0619, 2.0999],
#                                   up=[0.1204, -0.9852, 0.1215])











































































