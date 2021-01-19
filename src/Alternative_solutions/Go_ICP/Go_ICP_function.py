#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 14 14:28:44 2021

@author: sebastianhedegaardhansen
"""
import numpy as np;
import open3d as o3d
import numpy as np
import time 
import copy
import Go_ICP_function as icpf
from py_goicp import GoICP, POINT3D, ROTNODE, TRANSNODE


myparams = None
myconfiguration_file = None
def find_and_delete_planes(original_cluster_cloud, 
                           ddistance_threshold=0.1,
                           rransac_n=3,
                           nnum_iterations=1000,
                           visualize_on = True):
    """
    returns the point cloud cleaned of planes 
    """
    cluster_cloud = copy.deepcopy(original_cluster_cloud)
    continue_statement = "y"
    delete_statement = False
    
    list_explored_planes = []
    plane_to_explore = False
    
    
    while continue_statement == "y":
    
        while plane_to_explore == False:
            plane_model, inliers = cluster_cloud.segment_plane(distance_threshold=ddistance_threshold,
                                                               ransac_n=rransac_n,
                                                               num_iterations=nnum_iterations
                                                              )
            [a, b, c, d] = plane_model
            
            # if this plane has already been checked, continue changing the parameters
            if [a, b, c, d] in list_explored_planes:
                rransac_n +=0.01
                num_iterations +=1
            
            # mark the plane as now explored and continue
            else: 
                list_explored_planes.append([a, b, c, d])
                plane_to_explore = True
                
                
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = cluster_cloud.select_by_index(inliers)
        print ("plane detected contains %s points" %len(inlier_cloud.points))
        inlier_cloud.paint_uniform_color([1.0, 0, 0]) # points on the plane
        temp_cluster_cloud = copy.deepcopy(cluster_cloud)
        outlier_cloud = temp_cluster_cloud.select_by_index(inliers, invert=True)

        #o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

        if visualize_on == True:
            custom_draw_geometry(inlier_cloud+outlier_cloud, 
                             mytitle = "biggest_cluster_cloud_and_outliers", mytuples = "",
                             params =myparams, 
                             configuration_file = myconfiguration_file, 
                             take_screen_shot = False,
                             rotate = True)
            
        print('Delete the plane in red (y/n): ')
        delete_statement = input()
        if str(delete_statement) == "y":
            cluster_cloud = outlier_cloud
        
        print('continue finding planes (y/n): ')
        continue_statement = input()
   
    return cluster_cloud

def custom_draw_geometry(pcd, 
                         mytitle = None, mytuples = None,
                         params =None, 
                         configuration_file = None, 
                         take_screen_shot = False,
                         rotate = False,
                         onewindow = False
                        ):
    # The following code achieves the same effect as:
    # o3d.visualization.draw_geometries([pcd])
    
    if isinstance(pcd,list):
        
        #count of point clouds
        l = len(pcd)
        
        if isinstance(mytitle,list):
            mmytitle = mytitle
        else:
            mmytitle = ["custom draw geometry of pcl_"+str(i+1) for i in list(range(l))]
            
        if isinstance(mytuples,list):
            mmytuples = mytuples
        else:
            mmytuples = [None]*l
        
        if onewindow == False: 
            for i in list(range(len(pcd))):
                custom_draw_geometry(pcd[i], 
                                 mytitle = mmytitle[i], mytuples = mmytuples[i],
                                 params =params, 
                                 configuration_file =configuration_file, 
                                 take_screen_shot = take_screen_shot,
                                 rotate = rotate)
        else:
            all_pc = sum([pcd[i] for i in range(l)])
            custom_draw_geometry(all_pc, 
                 mytitle = mmytitle[i], mytuples = mmytuples[i],
                 params =params, 
                 configuration_file =configuration_file, 
                 take_screen_shot = take_screen_shot,
                 rotate = rotate)
            
    
    
    else: 
        
        # 1- initialize the visualizer
        vis = o3d.visualization.Visualizer()

        #2- set up naming system
        if mytitle is None:
            mytitle = "custom_draw_geometry "

        if mytuples is not None:
            mytitle = create_title(mytitle, mytuples)

        # 3- create widow, otherwise the kernel crashes
        vis.create_window(window_name=mytitle, 
                  width=1000, height=800, 
                  left=50, top=50, 
                  visible=True)

        # 4- add the geometry before taking view control
        #pcd.paint_uniform_color([0.3, 0.3, 0]) 
        vis.add_geometry(pcd)

        # 5- take view control after having added the geometry and before vis.run
        ctr = vis.get_view_control()
        if params is not None: 
            #print ("loading parameters: \n ",params)
            parameters = o3d.io.read_pinhole_camera_parameters(params)
            ctr.convert_from_pinhole_camera_parameters(parameters)

        if configuration_file is not None:
            #print ("loading configuration file: \n ",configuration_file)
            vis.get_render_option().load_from_json(str(configuration_file)) 

        if rotate == True:      
            def rotate_view(vis):
                ctr = vis.get_view_control()
                ctr.rotate(5.0, 0.0) #(speed, direction? )
                return False

            o3d.visualization.draw_geometries_with_animation_callback([pcd],
                                                               rotate_view,
                                                               window_name=mytitle, 
                                                               width=1000, height=800, 
                                                               left=50, top=50)
        elif rotate == "interactive":
        
            def change_background_to_black(vis):
                opt = vis.get_render_option()
                opt.background_color = np.asarray([0, 0, 0])
                return False
            
            def change_background_to_white(vis):
                opt = vis.get_render_option()
                opt.background_color = np.asarray([255, 255, 255])
                return False

            def rotate_view(vis):
                ctr = vis.get_view_control()
                ctr.rotate(10.0, 0.0) #(speed, direction? )
                return False

            def load_render_option(vis):
                vis.get_render_option().load_from_json(
                    myconfiguration_file)
                return False
                                                                      
            def plot_axes(vis):
                opt = vis.get_render_option()
                opt.show_coordinate_frame = not opt.show_coordinate_frame                                                    
                return False
                                                                    

            key_to_callback = {}
            key_to_callback[ord("R")] = load_render_option
            key_to_callback[ord("S")] = rotate_view
            key_to_callback[ord("B")] = change_background_to_black
            key_to_callback[ord("W")] = change_background_to_white
            key_to_callback[ord("A")] = plot_axes
            

            o3d.visualization.draw_geometries_with_key_callbacks([pcd],
                                                                 key_to_callback,                                                                                                                             
                                                                 window_name=mytitle, 
                                                                 width=1000, height=800, 
                                                                 left=50, top=50, 
                                                                 )                                                     
        else: 
            
            vis.run()




        #vis.run()

        if take_screen_shot == True:
            #print ("snipping")
            vis.capture_screen_image(mytitle+'.png')

        vis.destroy_window()
        
def create_title(mytitle, mytuples):
    mytitle = mytitle+"-"
    for i in range(len(mytuples)):
            mytitle = mytitle+str(mytuples[i][0])+"="+str(mytuples[i][1])+"-"
    mytitle = mytitle[:-1]
    return mytitle

def demo_crop_geometry(pcd):
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    temp_pcl = copy.deepcopy(pcd)
    o3d.visualization.draw_geometries_with_editing([temp_pcl])