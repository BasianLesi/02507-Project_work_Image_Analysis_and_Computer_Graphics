
import open3d as o3d

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



def pick_points(pcd):
    #print("")
    #print("1) Please pick at least three correspondences using [shift + left click]")
    #print("   Press [shift + right click] to undo point picking")
    #print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window("select 3 points with [shift + left click]; close view with [Q]",
                      width=1000, height=800, 
                      left=50, top=50, 
                      visible=True)
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()