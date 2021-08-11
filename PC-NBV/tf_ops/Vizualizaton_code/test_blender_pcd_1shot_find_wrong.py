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

    viewspace_path = '/home/alex-pop/Desktop/Doctorat/Blender_visualization/viewspace_shapenet_33.txt'

    test_viewstate_nochange_path ='/home/alex-pop/Desktop/Doctorat/Blender_visualization/FIles/Permuted_vs_nochange/Test_viewstate_nochange.txt'
    test_viewstate_permut_path ='/home/alex-pop/Desktop/Doctorat/Blender_visualization/FIles/Permuted_vs_nochange/Test_viewstate_permuted.txt'

    
    output_dir = '/home/alex-pop/Desktop/Doctorat/Blender_visualization'

    nr_views=16

    


    data_type = 'test'
    class_list_path = '/home/alex-pop/Desktop/Doctorat/Backups/Trial_Test_Valid_mat/Train_Test/' + data_type + '/_class.txt'
    ShapeNetv1_dir = '/home/alex-pop/Desktop/Doctorat/Backups/Trial_Test_Valid_mat/Train_Test/'
    with open(os.path.join(class_list_path)) as file:
        class_list = [line.strip() for line in file]

    #print(class_list)



    viewspace = np.loadtxt(viewspace_path)
    
    test_viewstate_nochange=np.loadtxt(test_viewstate_nochange_path)
    test_viewstate_permut=np.loadtxt(test_viewstate_permut_path)

    width = 640
    height = 480
    focal = 238 * 2

    scene, camera, output = setup_blender(width, height, focal, output_dir)
    intrinsics = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])


    np.savetxt(os.path.join(output_dir, 'intrinsics.txt'), intrinsics, '%f')

    viewspace_start_filepath='/home/alex-pop/Desktop/Doctorat/Blender_visualization/Viewspace_start.txt'
    nr_pozitie_filepath='/home/alex-pop/Desktop/Doctorat/Blender_visualization/nr_pozitie.txt'
    tip_view_path='/home/alex-pop/Desktop/Doctorat/Blender_visualization/tip_view.txt'
    model_view_path='/home/alex-pop/Desktop/Doctorat/Blender_visualization/FIles/Permuted_vs_nochange/Test_viewstate_model_permuted.txt'

    viewsapace_start=np.loadtxt(viewspace_start_filepath)
    nr_pozitie=np.loadtxt(nr_pozitie_filepath)

    with open(os.path.join(model_view_path)) as file:
        model_id_list = [line.strip() for line in file]

    past_model='no_model'
    
    

    bpy.data.objects['Camera'].select=False
    bpy.data.objects['Camera'].select=True

    problem_views_path="/home/alex-pop/Desktop/Doctorat/Blender_visualization/FIles/Permuted_vs_nochange/Problem_views.txt"
    problem_views=np.loadtxt(problem_views_path)

    length_problem_views=int(np.size(problem_views))
    
    
    if(int(viewsapace_start)==0):
        print('restart')
        vs_start=open('/home/alex-pop/Desktop/Doctorat/Blender_visualization/Viewspace_start.txt', 'w+')
        vs_start.write("1")
        vs_start.close()
        nr_poz=open('/home/alex-pop/Desktop/Doctorat/Blender_visualization/nr_pozitie.txt', 'w+')
        nr_poz.write('0')
        nr_poz.close()

        for m in bpy.data.meshes:
            bpy.data.meshes.remove(m)
        for m in bpy.data.materials:
            m.user_clear()
            bpy.data.materials.remove(m)

        j=0



    else:
        j=int(nr_pozitie)

        if(j<length_problem_views):
            if(j//3==0):
                operation_type="Differences_permut_bad"  
                print(operation_type) 
            if(j//3==1):
                operation_type="Differences_nochange_bad" 
                print(operation_type) 
            if(j//3==2):
                operation_type="Coverage_nochange_bad" 
                print(operation_type) 
            if(j//3==3):
                operation_type="Coverage_permut_bad" 
                print(operation_type)  


            

            problem_views2=problem_views[:,0].astype(int)

            i=problem_views2[j]

            working_model=model_id_list[i]
            
            print("Nr_pozitie:"+str(i))

            pozitie_actuala=int(test_viewstate_nochange[i][0])
            pozitie_prezisa_nochange=int(test_viewstate_nochange[i][1]) 
            pozitie_greedy=int(test_viewstate_nochange[i][2]) 
            pozitie_prezisa_permut_brut=int(test_viewstate_nochange[i][2]) 

            pozitie_presiza_permut_back=(pozitie_prezisa_permut_brut +pozitie_actuala) % nr_views

            print(working_model+" "
                  "Pozitie actuala:"+str(pozitie_actuala)+" "
                  "Pozitie prezisa nochange:"+str(pozitie_prezisa_nochange)+" "
                  "Pozitie prezisa permut brut:"+str(pozitie_prezisa_permut_brut)+" "
                  "Pozitie prezisa permut back:"+str(pozitie_presiza_permut_back)+" "
                  "Pozitie prezisa Groundtruth:"+str(pozitie_greedy))                       


            output_dir_2=os.path.join(output_dir, operation_type)
            exr_dir = os.path.join(output_dir_2, 'exr',working_model, str(i%nr_views))
            pose_dir = os.path.join(output_dir_2, 'pose',working_model, str(i%nr_views))
            os.makedirs(exr_dir, exist_ok=True)
            os.makedirs(pose_dir, exist_ok=True)  

            pcd_dir = os.path.join(output_dir_2, 'pcd',working_model, str(i%nr_views))

            
            os.makedirs(pcd_dir, exist_ok=True)

            


            nr_poz=open('/home/alex-pop/Desktop/Doctorat/Blender_visualization/nr_pozitie.txt', 'w+')
                
            j=j+1
            print("New position:"+str(j))
            nr_poz.write(str(j))
            nr_poz.close()


            
           

            
    #       

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

            pozitie_actuala=int(test_viewstate_nochange[i][0])    
            cam_pose = mathutils.Vector((viewspace[pozitie_actuala][0], viewspace[pozitie_actuala][1], viewspace[pozitie_actuala][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            pose_matrix = camera.matrix_world
            output.file_slots[0].path = os.path.join(exr_dir,"actual"+".exr")
            bpy.ops.render.render(write_still=True)
            np.savetxt(os.path.join(pose_dir, "actual"+".txt"), pose_matrix, '%f') 
            pose_path=os.path.join(pose_dir, "actual"+".txt")

            exr_path_1 = os.path.join(exr_dir, "actual"+".exr")
            exr_path_2 = os.path.join(exr_dir, "actual"+".exr0001"+".exr")

                
                
            os.rename(exr_path_2, exr_path_1)
                
            depth = read_exr(exr_path_1, height, width)
            
            pose = np.loadtxt(pose_path)
            points1 = depth2pcd(depth, intrinsics, pose)
            if (points1.shape[0] == 0):
                points1 = np.array([(1.0,1.0,1.0)])
            pcd1 = open3d.geometry.PointCloud()
            pcd1.points = open3d.utility.Vector3dVector(points1)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,"1_actual"+".pcd"), pcd1)
        
            

            


            pozitie_prezisa=int(test_viewstate_nochange[i][1])    
            cam_pose = mathutils.Vector((viewspace[pozitie_prezisa][0], viewspace[pozitie_prezisa][1], viewspace[pozitie_prezisa][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            pose_matrix = camera.matrix_world
            output.file_slots[0].path = os.path.join(exr_dir, "predicted"+".exr")
            bpy.ops.render.render(write_still=True)
            np.savetxt(os.path.join(pose_dir, "predicted"+".txt"), pose_matrix, '%f') 
            pose_path=os.path.join(pose_dir, "predicted"+".txt")
                
            exr_path_1 = os.path.join(exr_dir, "predicted"+".exr")
            exr_path_2 = os.path.join(exr_dir, "predicted"+".exr0001"+".exr")

            os.rename(exr_path_2, exr_path_1)
            


            pose_path = os.path.join(pose_dir, "predicted"+".txt")  

            depth = read_exr(exr_path_1, height, width)
                
            pose = np.loadtxt(pose_path)
            points2 = depth2pcd(depth, intrinsics, pose)
            if (points2.shape[0] == 0):
                points2 = np.array([(1.0,1.0,1.0)])
            pcd2 = open3d.geometry.PointCloud()
            pcd2.points = open3d.utility.Vector3dVector(points2)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,"2_predicted"+".pcd"), pcd2)

            pcd_combined_predict=open3d.geometry.PointCloud()
            points_predict_combined=np.concatenate((points1,points2),axis=0)
            pcd_combined_predict.points=open3d.utility.Vector3dVector(points_predict_combined)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,"combined_predict"+".pcd"), pcd_combined_predict)


            pozitie_greedy=int(test_viewstate_nochange[i][2])    
            cam_pose = mathutils.Vector((viewspace[pozitie_greedy][0], viewspace[pozitie_greedy][1], viewspace[pozitie_greedy][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            pose_matrix = camera.matrix_world
            output.file_slots[0].path = os.path.join(exr_dir, "greedy"+".exr")
            bpy.ops.render.render(write_still=True)
            np.savetxt(os.path.join(pose_dir, "greedy"+".txt"), pose_matrix, '%f') 
            pose_path=os.path.join(pose_dir, "greedy"+".txt")
                
            exr_path_1 = os.path.join(exr_dir, "greedy"+".exr")
            exr_path_2 = os.path.join(exr_dir, "greedy"+".exr0001"+".exr")

            os.rename(exr_path_2, exr_path_1)
            


            pose_path = os.path.join(pose_dir,"greedy"+".txt")  

            depth = read_exr(exr_path_1, height, width)
                
            pose = np.loadtxt(pose_path)
            points3 = depth2pcd(depth, intrinsics, pose)
            if (points3.shape[0] == 0):
                points3 = np.array([(1.0,1.0,1.0)])
            pcd3 = open3d.geometry.PointCloud()
            pcd3.points = open3d.utility.Vector3dVector(points3)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,"4_greedy"+".pcd"), pcd3)

            pcd_combined_greedy=open3d.geometry.PointCloud()
            points_greedy_combined=np.concatenate((points1,points3),axis=0)
            pcd_combined_greedy.points=open3d.utility.Vector3dVector(points_greedy_combined)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,"combined_greedy"+".pcd"), pcd_combined_greedy)


            pozitie_prezisa_permut_brut=int(test_viewstate_permut[i][2]) 
            pozitie_presiza_permut_back=(pozitie_prezisa_permut_brut +pozitie_actuala) % nr_views
            cam_pose = mathutils.Vector((viewspace[pozitie_presiza_permut_back][0], viewspace[pozitie_presiza_permut_back][1], viewspace[pozitie_presiza_permut_back][2])) 
            center_pose = mathutils.Vector((0, 0, 0))
            direct = center_pose - cam_pose
            rot_quat = direct.to_track_quat('-Z', 'Y')
            camera.rotation_euler = rot_quat.to_euler()
            camera.location = cam_pose
            pose_matrix = camera.matrix_world
            output.file_slots[0].path = os.path.join(exr_dir, "permut"+".exr")
            bpy.ops.render.render(write_still=True)
            np.savetxt(os.path.join(pose_dir, "permut"+".txt"), pose_matrix, '%f') 
            pose_path=os.path.join(pose_dir, "permut"+".txt")
                
            exr_path_1 = os.path.join(exr_dir, "permut"+".exr")
            exr_path_2 = os.path.join(exr_dir, "permut"+".exr0001"+".exr")

            os.rename(exr_path_2, exr_path_1)
            


            pose_path = os.path.join(pose_dir,"permut"+".txt")  

            depth = read_exr(exr_path_1, height, width)
                
            pose = np.loadtxt(pose_path)
            points4 = depth2pcd(depth, intrinsics, pose)
            if (points3.shape[0] == 0):
                points3 = np.array([(1.0,1.0,1.0)])
            pcd4 = open3d.geometry.PointCloud()
            pcd4.points = open3d.utility.Vector3dVector(points4)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,"3_permut"+".pcd"), pcd4)

            pcd_permut_greedy=open3d.geometry.PointCloud()
            points_permut_combined=np.concatenate((points1,points4),axis=0)
            pcd_permut_greedy.points=open3d.utility.Vector3dVector(points_permut_combined)
            open3d.io.write_point_cloud(os.path.join(pcd_dir,"permut_greedy"+".pcd"), pcd_permut_greedy)
            


            
    #         nosame_dir = os.path.join(pcd_dir,'not_same')
    #         os.makedirs(nosame_dir, exist_ok=True)
        

    #         past_model=working_model
        

        # nr_poz=open('/home/alex-pop/Desktop/Doctorat/Blender_visualization/nr_pozitie.txt', 'w+')
            
        # j=j+1
        # print("New position:"+str(j))
        # nr_poz.write(str(j))
        # nr_poz.close()

        
        

              

           
