import open3d as o3d
import numpy as np
import copy
from param import *
from help_functions import *
import time
import math
import matplotlib.pyplot as plt

if __name__ == "__main__":
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    source_raw = o3d.io.read_point_cloud("../data/plyfolder/external/points0.ply")
    target_raw = o3d.io.read_point_cloud("../data/plyfolder/external/points02.ply")
    # source_raw = o3d.io.read_point_cloud("../stiched/upper.pcd")
    # target_raw = o3d.io.read_point_cloud("../stiched/external.pcd")
    source = source_raw.voxel_down_sample(voxel_size=0.2)
    target = target_raw.voxel_down_sample(voxel_size=0.2)
    # trans = [[0.862, 0.011, -0.507, 0.0], [-0.139, 0.967, -0.215, 0.7],
    #          [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]]

    source_down, target_down, source_fpfh, target_fpfh, processed_source, processed_target, trans_init= prepare_dataset(voxel_size, source_raw, target_raw)
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size, trans_init)


    source.transform(result_ransac.transformation)
    target.transform(result_ransac.transformation)

    flip_transform = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, -1]]
    source.transform(flip_transform)
    target.transform(flip_transform)

    source.paint_uniform_color([1, 0.706, 0])
    target.paint_uniform_color([0, 0.651, 0.929])


    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(source)
    vis.add_geometry(target)
    # threshold = 0.05
    icp_iteration = 100
    save_image = False

    radius_normal = voxel_size * 2
    distance_threshold = voxel_size 

    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))

    for i in range(30):
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)

    print("inlier_rmse = ",result_ransac.inlier_rmse)
    print("fitness = ",result_ransac.fitness)

    rmse = []
    fitness = []
    iteration = []

    for i in range(icp_iteration):
        result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))

        # reg_p2l = o3d.pipelines.registration.registration_icp(
        #     source, target, distance_threshold, result_ransac.transformation,
        #     o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
        source.transform(result.transformation)
        vis.update_geometry(source)
        vis.poll_events()
        vis.update_renderer()
        rmse.append(result.inlier_rmse)
        fitness.append(result.fitness/4)
        y = result.inlier_rmse
        fit = result.fitness
        iteration.append(i)
        # plt.scatter(i, y, color="blue")
        plt.plot(iteration, rmse, color = "red")
        plt.plot(iteration, fitness, color = "blue")
        plt.xlabel("iteration")
        plt.ylabel("rmse")
        plt.pause(0.0001)
        # draw_registration_result(source, target, result.transformation)
        if save_image:
            plt.savefig("images/plot/plot_%04d.jpg" % i)
            # vis.capture_screen_image("images/temp_%04d.jpg" % i)
    vis.destroy_window()

    