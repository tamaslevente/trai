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
import time




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

    viewspace_path = '/home/cuda/Alex/trai/PC-NBV/Vizualizaton_code/viewspace_shapenet_33.txt'

    test_predicted_path ='/home/cuda/Alex/trai/PC-NBV/Test_viewstate.txt'

    
    output_dir = '/home/cuda/Alex/trai/PC-NBV/Vizualizaton_code'


    viewspace = np.loadtxt(viewspace_path)
    test_viewstate=np.loadtxt(test_predicted_path)

    width = 640
    height = 480
    focal = 238 * 2

    scene, camera, output = setup_blender(width, height, focal, output_dir)
    intrinsics = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])


    np.savetxt(os.path.join(output_dir, 'intrinsics.txt'), intrinsics, '%f')


    exr_dir = os.path.join(output_dir, 'exr', "02654")
    pose_dir = os.path.join(output_dir, 'pose', "02654")
    os.makedirs(exr_dir, exist_ok=True)
    os.makedirs(pose_dir, exist_ok=True)   

    # bpy.data.objects['Camera'].select=False
    # bpy.data.objects['Camera'].select=True
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
    
    

    # for i in range(10):    
    #      trig_time=0
    #      trig_time = time.time()
         
    #      value=True
    #      ok=True

    #      while(value & ok):
    #         new_time=time.time()
    #         #print("Triger time: "+str(trig_time))
    #         #print("New time: "+str(new_time))
    #         #print("Difference: "+str(new_time-trig_time))
    #         if(new_time-trig_time>5):
    #             print("Triger time: "+str(trig_time))
    #             print("New time: "+str(new_time))
    #             print("Difference: "+str(new_time-trig_time))
    #             scene.frame_set(i)
    #             pozitie_prezisa=int(test_viewstate[i][1])    
    #             cam_pose = mathutils.Vector((viewspace[i][0], viewspace[i][1], viewspace[i][2])) 
    #             center_pose = mathutils.Vector((0, 0, 0))
    #             direct = center_pose - cam_pose
    #             rot_quat = direct.to_track_quat('-Z', 'Y')
    #             camera.rotation_euler = rot_quat.to_euler()
    #             camera.location = cam_pose
    #             ok=0
       
        
    

    

    for i in range(10):    
        scene.frame_set(i)
        pozitie_actuala=int(test_viewstate[i][0])
        print(pozitie_actuala)
        cam_pose = mathutils.Vector((viewspace[pozitie_actuala][0], viewspace[pozitie_actuala][1], viewspace[pozitie_actuala][2])) 
        center_pose = mathutils.Vector((0, 0, 0))
        direct = center_pose - cam_pose
        rot_quat = direct.to_track_quat('-Z', 'Y')
        camera.rotation_euler = rot_quat.to_euler()
        camera.location = cam_pose


        time.sleep(1)
        scene.frame_set(i)
        pozitie_prezisa=int(test_viewstate[i][1])    
        cam_pose = mathutils.Vector((viewspace[i][0], viewspace[i][1], viewspace[i][2])) 
        center_pose = mathutils.Vector((0, 0, 0))
        direct = center_pose - cam_pose
        rot_quat = direct.to_track_quat('-Z', 'Y')
        camera.rotation_euler = rot_quat.to_euler()
        camera.location = cam_pose
        time.sleep(1)                
       
        
        

    #     np.savetxt(os.path.join(pose_dir, '%d.txt' % i), pose_matrix, '%f') 

              

           
