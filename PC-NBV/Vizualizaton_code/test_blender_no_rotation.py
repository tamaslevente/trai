# Author: Wentao Yuan (wyuan1@cs.cmu.edu) 05/31/2018
# Modified by Rui Zeng 07/12/2020

import bpy
import mathutils
import numpy as np
import os
import sys
import time
import pdb
import argparse



def setup_blender(width, height, focal_length, output_dir):
    # camera
    camera = bpy.data.objects['Camera']
    camera.data.angle = np.arctan(width / 2 / focal_length) * 2
    # camera.data.clip_end = 1.2
    # camera.data.clip_start = 0.2

    
    

    

    # render layer
    scene = bpy.context.scene
    scene.render.filepath = 'buffer'
    scene.render.image_settings.color_depth = '16'
    scene.render.resolution_percentage = 100
    scene.render.resolution_x = width
    scene.render.resolution_y = height

    # compositor nodes
    scene.use_nodes = True
    tree = scene.node_tree
    rl = tree.nodes.new('CompositorNodeRLayers')
    output = tree.nodes.new('CompositorNodeOutputFile')
    output.base_path = ''
    output.format.file_format = 'OPEN_EXR'
    #tree.links.new(rl.outputs['Z'], output.inputs[0])
    tree.links.new(rl.outputs['Depth'], output.inputs[0]) #-Original

    # remove default cube
    # bpy.data.objects['Camera'].select=False
    # bpy.data.objects['Cube'].select = True
    # bpy.ops.object.delete()
    # bpy.data.objects['Camera'].select=True

    return scene, camera, output





if __name__ == '__main__':

    viewspace_path = '/home/alex-pop/Desktop/Doctorat/Program_blender/viewspace_shapenet_33.txt'

    test_predicted_path ='/home/alex-pop/Desktop/Doctorat/Program_blender/Test_viewstate.txt'

    
    output_dir = '/home/alex-pop/Desktop/Doctorat/Program_blender/'


    data_type = 'valid'
    class_list_path = '/home/alex-pop/Desktop/Doctorat/Backups/Trial_Test_Valid_mat/Train_Test/' + data_type + '/_class.txt'
    ShapeNetv1_dir = '/home/alex-pop/Desktop/Doctorat/Backups/Trial_Test_Valid_mat/Train_Test/'
    with open(os.path.join(class_list_path)) as file:
        class_list = [line.strip() for line in file]

    #print(class_list)


    
    # for class_id in self.class_list:
    #     model_list = os.listdir(os.path.join(ShapeNetv1_dir, self.data_type, class_id))
        #for model_id in model_list:




    viewspace = np.loadtxt(viewspace_path)
    test_viewstate=np.loadtxt(test_predicted_path)

    width = 640
    height = 480
    focal = 238 * 2

    scene, camera, output = setup_blender(width, height, focal, output_dir)
    intrinsics = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])


    np.savetxt(os.path.join(output_dir, 'intrinsics.txt'), intrinsics, '%f')

    viewspace_start_filepath='/home/alex-pop/Desktop/Doctorat/Program_blender/Viewspace_start.txt'
    nr_pozitie_filepath='/home/alex-pop/Desktop/Doctorat/Program_blender/nr_pozitie.txt'
    tip_view_path='/home/alex-pop/Desktop/Doctorat/Program_blender/tip_view.txt'
    model_view_path='/home/alex-pop/Desktop/Doctorat/Program_blender/Test_viewstate_model.txt'

    viewsapace_start=np.loadtxt(viewspace_start_filepath)
    nr_pozitie=np.loadtxt(nr_pozitie_filepath)
    tip_view=np.loadtxt(tip_view_path)

    with open(os.path.join(model_view_path)) as file:
        model_id_list = [line.strip() for line in file]

    print(model_id_list[0])




    # viewspace_start=
    # nr_pozitie=open('nr_pozitie.txt', 'w+')
    # tip_view=open('nr_pozitie.txt', 'w+')


    exr_dir = os.path.join(output_dir, 'exr', "02654")
    pose_dir = os.path.join(output_dir, 'pose', "02654")
    os.makedirs(exr_dir, exist_ok=True)
    os.makedirs(pose_dir, exist_ok=True)   

    bpy.data.objects['Camera'].select=False
    bpy.data.objects['Camera'].select=True
    #bpy.context.object.scan_type='kinect'
    #bpy.context.object.save_scan=True

    # Redirect output to log file
    

    # Rotate model by 90 degrees around x-axis (z-up => y-up) to match ShapeNet's coordinates
    #bpy.ops.transform.rotate(value=-np.pi / 2, axis=(1, 0, 0))  # ->original code



    #bpy.ops.transform.rotate(value=-np.pi / 2, orient_axis=(1, 0, 0)) 
    #bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    #bpy.ops.transform.rotate(value=-np.pi / 2, orient_axis='Z')

    # Render
    # for i in range(viewspace.shape[0]):
    
    t_view=open('/home/alex-pop/Desktop/Doctorat/Program_blender/tip_view.txt', 'w+')
    
    if(int(viewsapace_start)==0):
        print('restart')
        vs_start=open('Viewspace_start.txt', 'w+')
        nr_poz=open('nr_pozitie.txt', 'w+')
        vs_start.write("1")
        vs_start.close()
        nr_poz.write('0')
        nr_poz.close()
        t_view.write('0')
        t_view.close()
    else:
        i=int(nr_pozitie)

       






        if(int(tip_view)==0):
            pozitie_actuala=int(test_viewstate[i][0])    
            cam_pose = mathutils.Vector((viewspace[pozitie_actuala][0], viewspace[pozitie_actuala][1], viewspace[pozitie_actuala][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            t_view.write('1')
            t_view.close()

            print('Nr:'+str(i)+" Initial position nr:"+str(pozitie_actuala))
        else:

            nr_poz=open('nr_pozitie.txt', 'w+')

            pozitie_prezisa=int(test_viewstate[i][1])    
            cam_pose = mathutils.Vector((viewspace[pozitie_prezisa][0], viewspace[pozitie_prezisa][1], viewspace[pozitie_prezisa][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            t_view.write('0')
            t_view.close()
            print('Nr:'+str(i)+" Predicted position nr:"+str(pozitie_prezisa))
            i=i+1
            nr_poz.write(str(i))
            nr_poz.close()

            

        
    
    # i=0     
    # scene.frame_set(i)
    

        

    #     np.savetxt(os.path.join(pose_dir, '%d.txt' % i), pose_matrix, '%f') 

              

           
