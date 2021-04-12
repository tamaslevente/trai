# Author: Wentao Yuan (wyuan1@cs.cmu.edu) 05/31/2018
# Modified by Rui Zeng 07/12/2020

import bpy
import Imath
import OpenEXR
from open3d import *
import array
import mathutils
import numpy as np
import os
import sys
import time
import pdb
import argparse


def read_exr(exr_path, height, width):
    file = OpenEXR.InputFile(exr_path)
    depth_arr = array.array('f', file.channel('R', Imath.PixelType(Imath.PixelType.FLOAT)))
    depth = np.array(depth_arr).reshape((height, width))
    depth[depth < 0] = 0
    depth[np.isinf(depth)] = 0
    return depth


def depth2pcd(depth, intrinsics, pose):
    inv_K = np.linalg.inv(intrinsics)
    inv_K[2, 2] = -1
    depth = np.flipud(depth)
    y, x = np.where(depth > 0)
    # image coordinates -> camera coordinates
    points = np.dot(inv_K, np.stack([x, y, np.ones_like(x)] * depth[y, x], 0))
    # camera coordinates -> world coordinates
    points = np.dot(pose, np.concatenate([points, np.ones((1, points.shape[1]))], 0)).T[:, :3]
    return points




def setup_blender(width, height, focal_length, output_dir):
    # camera

    
    

    # camera_data = bpy.data.cameras.new(name='Camera')
    # camera_object = bpy.data.objects.new('Camera', camera_data)
    # bpy.context.scene.collection.objects.link(camera_object)



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


    data_type = 'test'
    class_list_path = '/home/alex-pop/Desktop/Doctorat/Backups/Trial_Test_Valid_mat/Train_Test/' + data_type + '/_class.txt'
    ShapeNetv1_dir = '/home/alex-pop/Desktop/Doctorat/Backups/Trial_Test_Valid_mat/Train_Test/'
    with open(os.path.join(class_list_path)) as file:
        class_list = [line.strip() for line in file]

    #print(class_list)


    
    


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

    past_model='no_model'
    

    




    # viewspace_start=
    # nr_pozitie=open('nr_pozitie.txt', 'w+')
    # tip_view=open('nr_pozitie.txt', 'w+')


    

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
        vs_start=open('/home/alex-pop/Desktop/Doctorat/Program_blender/Viewspace_start.txt', 'w+')
        nr_poz=open('/home/alex-pop/Desktop/Doctorat/Program_blender/nr_pozitie.txt', 'w+')
        vs_start.write("1")
        vs_start.close()
        nr_poz.write('0')
        nr_poz.close()
        t_view.write('0')
        t_view.close()


        for m in bpy.data.meshes:
            bpy.data.meshes.remove(m)
        for m in bpy.data.materials:
            m.user_clear()
            bpy.data.materials.remove(m)

       



    else:
        i=int(nr_pozitie)
        print("Nr_pozitie:"+str(i))

        exr_dir = os.path.join(output_dir, 'exr', str(i))
        pose_dir = os.path.join(output_dir, 'pose', str(i))
        os.makedirs(exr_dir, exist_ok=True)
        os.makedirs(pose_dir, exist_ok=True)  

        #depth_dir = os.path.join(output_dir, 'depth', str(i))
        pcd_dir = os.path.join(output_dir, 'pcd', str(i))

        #os.makedirs(depth_dir, exist_ok=True)
        os.makedirs(pcd_dir, exist_ok=True)

        if((i==0) and int(tip_view)==0):
            past_model='no model'
        elif(i==0):
            past_model=model_id_list[i]
        else:
            past_model=model_id_list[i-1]

        working_model=model_id_list[i]
        print(model_id_list[i])

        if(past_model!=working_model):
            for m in bpy.data.meshes:
                bpy.data.meshes.remove(m)
            for m in bpy.data.materials:
                m.user_clear()
                bpy.data.materials.remove(m)
       
        for class_id in class_list:
            model_list = os.listdir(os.path.join(ShapeNetv1_dir, data_type, class_id))
            if(past_model!=working_model):
                if(working_model in model_list):
 

                    model_path = os.path.join(ShapeNetv1_dir, data_type, class_id, working_model, 'model.obj')
                    bpy.ops.import_scene.obj(filepath=model_path)

                    bpy.ops.transform.rotate(value=-np.pi / 2, axis=(1, 0, 0))

                    break

            
        

        past_model=working_model

       



        if(int(tip_view)==0):
            pozitie_actuala=int(test_viewstate[i][0])    
            cam_pose = mathutils.Vector((viewspace[pozitie_actuala][0], viewspace[pozitie_actuala][1], viewspace[pozitie_actuala][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            pose_matrix = camera.matrix_world

             



            output.file_slots[0].path = os.path.join(exr_dir, str(i)+"_"+str(int(tip_view))+".exr")
            bpy.ops.render.render(write_still=True)
            np.savetxt(os.path.join(pose_dir, str(i)+"_"+str(int(tip_view))+".txt"), pose_matrix, '%f') 
            pose_path=os.path.join(pose_dir, str(i)+"_"+str(int(tip_view))+".txt")

            exr_path_1 = os.path.join(exr_dir, str(i)+"_"+str(int(tip_view))+".exr")
            exr_path_2 = os.path.join(exr_dir, str(i)+"_"+str(int(tip_view))+".exr0001"+".exr")

            
               
            os.rename(exr_path_2, exr_path_1)
            
            depth = read_exr(exr_path_1, height, width)
            depth_img = open3d.geometry.Image(np.uint16(depth * 100000))
            #open3d.io.write_image(os.path.join(depth_dir,str(i)+"_"+str(int(tip_view))+".png"), depth_img)

            pose = np.loadtxt(pose_path)
            points = depth2pcd(depth, intrinsics, pose)
            if (points.shape[0] == 0):
                points = np.array([(1.0,1.0,1.0)])
            pcd = open3d.geometry.PointCloud()
            pcd.points = open3d.utility.Vector3dVector(points)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,str(i)+"_"+str(int(tip_view))+".pcd"), pcd)



            t_view.write('1')
            t_view.close()

            print('Nr:'+str(i)+" Initial position nr:"+str(pozitie_actuala))
        else:

            nr_poz=open('/home/alex-pop/Desktop/Doctorat/Program_blender/nr_pozitie.txt', 'w+')

            pozitie_prezisa=int(test_viewstate[i][1])    
            cam_pose = mathutils.Vector((viewspace[pozitie_prezisa][0], viewspace[pozitie_prezisa][1], viewspace[pozitie_prezisa][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            pose_matrix = camera.matrix_world
            output.file_slots[0].path = os.path.join(exr_dir, str(i)+"_"+str(int(tip_view))+".exr")
            bpy.ops.render.render(write_still=True)
            np.savetxt(os.path.join(pose_dir, str(i)+"_"+str(int(tip_view))+".txt"), pose_matrix, '%f') 
            pose_path=os.path.join(pose_dir, str(i)+"_"+str(int(tip_view))+".txt")
            
            exr_path_1 = os.path.join(exr_dir, str(i)+"_"+str(int(tip_view))+".exr")
            exr_path_2 = os.path.join(exr_dir, str(i)+"_"+str(int(tip_view))+".exr0001"+".exr")

            os.rename(exr_path_2, exr_path_1)
           


            pose_path = os.path.join(pose_dir, str(i)+"_"+str(int(tip_view))+".txt")  

            depth = read_exr(exr_path_1, height, width)
            depth_img = open3d.geometry.Image(np.uint16(depth * 100000))
            #open3d.io.write_image(os.path.join(depth_dir,str(i)+"_"+str(int(tip_view))+".png"), depth_img)
            
            pose = np.loadtxt(pose_path)
            points = depth2pcd(depth, intrinsics, pose)
            if (points.shape[0] == 0):
                points = np.array([(1.0,1.0,1.0)])
            pcd = open3d.geometry.PointCloud()
            pcd.points = open3d.utility.Vector3dVector(points)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,str(i)+"_"+str(int(tip_view))+".pcd"), pcd)

            
            
            t_view.write('0')
            t_view.close()
            print('Nr:'+str(i)+" Predicted position nr:"+str(pozitie_prezisa))
            i=i+1
            nr_poz.write(str(i))
            nr_poz.close()

        # Clean up
        

            

        
    
    # i=0     
    # scene.frame_set(i)
    

        

    #     np.savetxt(os.path.join(pose_dir, '%d.txt' % i), pose_matrix, '%f') 

              

           
