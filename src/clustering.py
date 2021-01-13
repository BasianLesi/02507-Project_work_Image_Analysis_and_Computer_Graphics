import numpy as np
import open3d as o3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from visualization_functions import *
import copy
from param import *
import pickle as pkl
from help_functions import *
import statistics 
from statistics import mode


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


# filename = "../stiched/final.pcd"
# pcd =  o3d.io.read_point_cloud(filename)

# custom_draw_geometry(pcd)

# cluster_pcd = clustering(pcd)

# custom_draw_geometry(cluster_pcd)