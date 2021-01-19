import numpy as np
import open3d as o3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from visualization_functions import *
import copy
import pickle as pkl
from help_functions import *
import statistics 
from statistics import mode
from skspatial.objects import Points, Plane
from skspatial.plotting import plot_3d


def create_title(mytitle, mytuples):
    mytitle = mytitle+"-"
    for i in range(len(mytuples)):
            mytitle = mytitle+str(mytuples[i][0])+"="+str(mytuples[i][1])+"-"
    mytitle = mytitle[:-1]
    return mytitle

def most_common(List): 
    return(mode(List))

def clustering( original_cloud,
                myeps=0.2, 
                mymin_points=10,
               
               mytitle = "clustering",
               params =None, 
               configuration_file = None, 
               take_screen_shot = False,
               rotate = False,
               
               #statements
                print_statement = True,
                visualization_on = False):
    """
    eps = max Euclidean distance btw 2 points is same cluster (chose small epsilon to avoid long comptime)
    min_points = the minimal number of points necessary to create a cluster

    """
    cloud = copy.deepcopy(original_cloud)
     
    mytuples = list(zip(("myeps","mymin_points"),(myeps,mymin_points)))
    #if mytuples is not None:
    mytitle = create_title(mytitle, mytuples)
    
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(cloud.cluster_dbscan(eps=myeps, min_points=mymin_points, print_progress=print_statement))
        #labels = np.array(cloud.cluster_dbscan(eps=0.2, min_points=10, print_progress=print_statement))

    max_label = labels.max()
    
    ##identify the biggest cluster label
    biggest_cluster_label = most_common(labels)
    biggest_cluster_labels = [label for label in labels if label == biggest_cluster_label]
    ## select indexes of points in the biggest cluster
    index_biggest_cluster_labels = [i for i, e in enumerate(labels) if e == biggest_cluster_label]
    
    ##identify the outliers 
    negative_labels = [label for label in labels if label <0]
    
    ## set colors
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    #colors[labels != biggest_cluster_label] = 0 # in black everything that is not main cluster..
    colors[labels < 0] = 0 # in black the outliers...

    ##float64 array of shape (num_points, 3), range [0, 1] , use numpy.asarray() to access data
    cloud.colors = o3d.utility.Vector3dVector(colors[:, :3]) 
    
    
    if visualization_on == True: 
        #o3d.visualization.draw_geometries([cloud])
        
        custom_draw_geometry(cloud, 
                             mytitle = mytitle, mytuples = mytuples,
                             params =myparams, 
                             configuration_file = myconfiguration_file, 
                             take_screen_shot = take_screen_shot,
                             rotate = rotate
                            )

    if print_statement == True:
        print ("")
        print (f"point cloud has {max_label + 1} clusters")
        print (f"there are {len(labels)} points in all clusters")
        print (f"there are {len(negative_labels)} points identified as ouliers ")
        print (f"there are {len(biggest_cluster_labels)} points in the biggest cluster ")
    
    ## select only the biggest cluster
    biggest_cluster_cloud = cloud.select_by_index(index_biggest_cluster_labels)
    
    
    return biggest_cluster_cloud



def is_point_in_plane(point = [0,0,0],
                      plane_parameters = [0,0,0,0],
                      neg_margin = 0,
                      pos_margin = 0):

    d_point = sum([point[i]*plane_parameters[i] for i in range(3)])
    
    #print (d)
    #print (d_point)
    
    if d - neg_margin  <= d_point <= d+pos_margin:
        return True
    else:
        return False



def custom_remove_plane(original_cloud,
                        nneg_margin = 0.2,
                        ppos_margin = 0.01,
                        
                        #visualization parameters
                        mytitle = "custom_remove_planes", mytuples = None, 
                        params = None, #camera parameters,json file (P)
                        fov_step  = None, 
                        configuration_file = None, #object properties ,json file (O)
                        rotate = False,
                        
                        #statements
                        print_statements = True,
                        visualization_on = False,
                       ):
    """
    cloud = point cloud
    """
    cloud = copy.deepcopy(original_cloud)
    
    ## 
    mytuples = list(zip(("pos_margin","neg_margin"),(ppos_margin,nneg_margin)))
    
    # initialize empty list
    selected_points = []
      
    # print instructions
    if print_statements == True:
        print("")
        print("1) Please pick at least 3 points using [shift + left click]")
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to close the window")
        
    # we need 3 points
    while len(selected_points) != 3: 
        
        # selected points
        # in theory one can select any number of points, but we want three to find the plane
        selected_points = pick_points(cloud)
    
    # access the coordinates of the points
    # we want only 3 points to find the plane
    coordinates_points= [list(np.asarray(cloud.points)[selected_points[i]]) for i in range(3)]
    
    
    # translate in scikit language the newly found coordinates
    points = Points(coordinates_points)
    #identify the plane through the points
    plane = Plane.best_fit(points)

    # for each point in the point cloud, 
    # if it lies on the plane identified by the manually selected 3 points, it gets excluded
    all_points = np.asarray(cloud.points)
    all_indexes = list(range(len(all_points)))
    all_list_points = [list(e) for i, e in enumerate(all_points[:])]
    inliers = [list(all_points[i]) for i in range(len(all_points)) if is_point_in_plane(all_points[i], 
                                                                                      plane.normal,
                                                                                      neg_margin = nneg_margin,
                                                                                      pos_margin = ppos_margin)]
    
                                                                                            
    if print_statements == True:
        print ("")
        print ("plane point: ",plane.point)
        print ("normal to the plane: ",plane.normal) 
        print ("")
        print ("total number of points: ",len(all_points))                                                                                 
        print ("number excluded points : ",len(inliers))
        print ("with pos_margin= %s ; neg_margin= %s" %(ppos_margin,nneg_margin))
        print ("")
        print ("calculating indexes (might take a while...)")
    
    ## find the indexes of the outliers
    #inliers_idx = [i for i, e in enumerate(all_list_points) if e in inliers] #slower
    inliers_idx = [all_list_points.index(e) for e in inliers] #faster
    
    # include the invert of the outliers indexes
    temp_cloud= copy.deepcopy(cloud)
    cloud_new = temp_cloud.select_by_index(inliers_idx, invert=True)
    
    #visualize eventually
    if visualization_on == True:
        
        continue_statement = "y"
        
        while continue_statement == "y":
            
            mytuples,inliers_idx,update_p = update_parameters(mytuples,all_points,plane.normal,inliers_idx)
            
            if update_p == "y":
                # include the invert of the outliers indexes
                temp_cloud= copy.deepcopy(cloud)
                cloud_new = temp_cloud.select_by_index(inliers_idx, invert=True)
        
            #o3d.visualization.draw_geometries([cloud_new])
            #display_inlier_outlier(cloud, inliers_idx)
            custom_draw_geometry_outliers(cloud_new, inliers_idx, 
                                          mytitle = mytitle, mytuples = mytuples, 
                                          params = params, #camera parameters,json file (P)
                                          fov_step  = None, 
                                          configuration_file = configuration_file, #object properties ,json file (O)
                                          rotate = rotate)


            print('Delete the space in red (y/n): ')
            delete_statement = input()

            print('continue finding planes (y/n): ')
            continue_statement = input()
            
            

        if str(delete_statement) == "y":
            return cloud_new 

        else:
            print ("no points deleted")
            return cloud


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
                             mytitle = "biggest_cluster_cloud_and_outliers", mytuples = None,
                             params = None, 
                             configuration_file = None, 
                             take_screen_shot = False,
                             rotate = False)
            
        print('Delete the plane in red (y/n): ')
        delete_statement = input()
        if str(delete_statement) == "y":
            cluster_cloud = outlier_cloud
        
        print('continue finding planes (y/n): ')
        continue_statement = input()
   
    return cluster_cloud