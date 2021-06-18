import numpy as np
import bpy
import mathutils
import datetime
import math
import random 
import socket
import pickle

#bpy.ops.transform.rotate(value=-np.pi / 2, axis=(1, 0, 0))

#bpy.context.space_data.context='OBJECT'




Length=0.57
Width=0.14
Height=0.58

scaling=1



nr_executions=1





bpy.data.objects['Cube'].rotation_euler[0]=0
bpy.data.objects['Cube'].rotation_euler[1]=0
bpy.data.objects['Cube'].rotation_euler[2]=0

bpy.data.objects['Cube'].scale[0]=Length/scaling
bpy.data.objects['Cube'].scale[1]=Width/scaling
bpy.data.objects['Cube'].scale[2]=Height/scaling




bpy.context.object.location[0]=0
bpy.context.object.location[1]=0
bpy.context.object.location[2]= Height/scaling



z_object=Height/scaling




distance_to_corner=[1,1.3]

height_angle=[math.tan(math.pi / 3.5),math.tan(math.pi / 2.9)]




for i in range(1,2):
    dist_choice=i
    for j in range(0,1):
        angle_choice=j
        for k in range(0,1):
            for t in range(nr_executions):
                
                
                corner_choice=k

                error_position_x= random.uniform(-0.1, 0.1)
                error_position_y=random.uniform(-0.1, 0.1)
                error_position_z=random.uniform(0, 0.05)

                error_orientation_x=random.uniform(-0.1, 0.1)
                error_orientation_y=random.uniform(-0.1, 0.1)
                error_orientation_z=random.uniform(0, 0.1)

                error_rotation_x=random.uniform(-0.05, 0.05)
                error_rotation_y=random.uniform(-0.05, 0.05)
                error_rotation_z=random.uniform(0, 0.1)



                bpy.ops.object.select_all(action='DESELECT')

                for item in bpy.data.objects:
                    if item.type == 'MESH' and item.name.startswith('Scan'):
                        print(item)
                        bpy.data.objects.remove(bpy.data.objects[item.name], do_unlink=True) 

                for item in bpy.data.objects:
                    if item.type == 'MESH' and item.name.startswith('Noisy'):
                        print(item)
                        bpy.data.objects.remove(bpy.data.objects[item.name], do_unlink=True)    


                

            
                

                corners = np.array([[1, 1, 1], 
                                    [1, -1, 1],
                                    [-1, 1, 1], 
                                    [-1, -1, 1],
                                    [1, 1,-1],
                                    [1, -1, -1],
                                    [-1, 1, -1],
                                    [-1, -1,-1]])

                bpy.ops.blensor.delete_scans()

                
                
                

                print(dist_choice)
                print(angle_choice)
                print(corner_choice)


                

                bpy.data.objects['Camera'].select=False
                bpy.data.objects['Camera'].select=True

                bpy.context.object.scan_type='kinect'
                bpy.context.object.save_scan=True

                
                

                X_camera =(Length + (Length*distance_to_corner[dist_choice])/(Length*Length + Width*Width)) *corners[corner_choice][0]+error_position_x
                Y_camera =(Width + (Width*distance_to_corner[dist_choice])/(Length*Length + Width*Width)) *corners[corner_choice][1]+error_position_y
                Z_camera=z_object+(Height + height_angle[angle_choice]*distance_to_corner[dist_choice]) *corners[corner_choice][2]+error_position_z

               
                # X_camera =(Length/2 + (Length*distance_to_corner[dist_choice])/(Length*Length + Width*Width)) *corners[corner_choice][0]+error_position_x
                # Y_camera =(Width/2 + (Width*distance_to_corner[dist_choice])/(Length*Length + Width*Width)) *corners[corner_choice][1]+error_position_y
                # Z_camera=z_object+(Height/2 + height_angle[angle_choice]*distance_to_corner[dist_choice]) *corners[corner_choice][2]+error_position_z



                

                camera = bpy.data.objects['Camera']

                cam_pose = mathutils.Vector((X_camera, Y_camera, Z_camera))



                

                center_pose = mathutils.Vector((Length * corners[corner_choice][0] +error_orientation_x, Width*corners[corner_choice][1] +error_orientation_y, z_object+Height*corners[corner_choice][2] +error_orientation_z))
                #center_pose = mathutils.Vector((Length/2 * corners[corner_choice][0] +error_orientation_x, Width/2*corners[corner_choice][1] +error_orientation_y, z_object+Height/2*corners[corner_choice][2] +error_orientation_z))

                direct = center_pose - cam_pose
                rot_quat = direct.to_track_quat('-Z', 'Y')

                camera.rotation_euler = rot_quat.to_euler()
                camera.location = cam_pose


                Rotation_x=camera.rotation_euler[0]
                Rotation_y=camera.rotation_euler[1]
                Rotation_z=camera.rotation_euler[2]

                camera.rotation_euler[0]=Rotation_x+error_rotation_x
                camera.rotation_euler[1]=Rotation_y+error_rotation_y
                camera.rotation_euler[2]=Rotation_z+error_rotation_z

                # bpy.context.object.add_noise_scan_mesh=True
                bpy.ops.blensor.scan()

                i = 0; #Storage point cloud number
                point_cloud_list = []
                for item in bpy.data.objects:
                    if item.type == 'MESH' and item.name.startswith('Scan'):
                        print("Baking the point cloud... Patience, my friend!")
                        for sp in item.data.vertices:
                            #print('X=%+#5.3f\tY=%+#5.3f\tZ=%+#5.3f' % (sp.co[0], sp.co[1],sp.co[2]));
                            if(  (np.isnan(sp.co[0])==0) and(np.isnan(sp.co[1])==0) and (np.isnan(sp.co[2])==0) ):
                                point_cloud_list.append([sp.co[0], sp.co[1], sp.co[2]])
                                # string_out='%#5.3f\t%#5.3f\t%#5.3f \n' % (sp.co[0], sp.co[1],sp.co[2]);
                                # i = i+1;
                                # f.write(string_out);
                
                point_cloud = np.asarray(point_cloud_list)
                print('Point cloud shape is: ',point_cloud.shape)

                print('Send in the clowns... I mean, clouds!')
                HOST = '127.0.0.1'  # The server's hostname or IP address
                PORT = 65432        # The port used by the server
                serialized_cloud = pickle.dumps(point_cloud)
                print(serialized_cloud.__len__()) 
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((HOST, PORT))
                    print("Sending data...")
                    s.sendall(serialized_cloud)
                    print("Data sent!")
                    # data = s.recv(1024)

                # print('Received', repr(data))


                # now = datetime.datetime.now()
                # current_time = now.strftime("%Y%m%d%H%M%S")

                # path_file="/home/alex-pop/Desktop/DAte/cl_pcd_"+str(dist_choice)+"_"+str(angle_choice)+"_"+ str(corner_choice)+"_"+str(current_time)+"_"+str(t)+".pcd"
                # path_file_2="/home/alex-pop/Desktop/DAte/no_pcd_"+str(dist_choice)+"_"+str(angle_choice)+"_"+ str(corner_choice)+"_"+str(current_time)+"_"+str(t)+".pcd"

                # f=open(path_file,"w") #File location to modify
                # i = 0; #Storage point cloud number
                # for item in bpy.data.objects:
                #     if item.type == 'MESH' and item.name.startswith('Scan'):
                #         print('write once')
                #         for sp in item.data.vertices:
                #             #print('X=%+#5.3f\tY=%+#5.3f\tZ=%+#5.3f' % (sp.co[0], sp.co[1],sp.co[2]));
                #             if(  (np.isnan(sp.co[0])==0) and(np.isnan(sp.co[1])==0) and (np.isnan(sp.co[2])==0) ):
                #                 string_out='%#5.3f\t%#5.3f\t%#5.3f \n' % (sp.co[0], sp.co[1],sp.co[2]);
                #                 i = i+1;
                #                 f.write(string_out);  
                # f.close()          
                # f=open(path_file,"r+") #File location to modify
                # new_data = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" %(i,i))
                # old = f.read()
                # f.seek(0)
                # f.write(new_data)
                # f.write(old )

                # f=open(path_file_2,"w") #File location to modify
                # i = 0; #Storage point cloud number
                # for item in bpy.data.objects:
                #     if item.type == 'MESH' and item.name.startswith('Noisy'):
                #         print('write once')
                #         for sp in item.data.vertices:
                #             #print('X=%+#5.3f\tY=%+#5.3f\tZ=%+#5.3f' % (sp.co[0], sp.co[1],sp.co[2]));
                #             string_out='%#5.3f\t%#5.3f\t%#5.3f \n' % (sp.co[0], sp.co[1],sp.co[2]);
                #             i = i+1;
                #             f.write(string_out);  
                # f.close()          
                # f=open(path_file_2,"r+") #File location to modify
                # new_data = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" %(i,i))
                # old = f.read()
                # f.seek(0)
                # f.write(new_data)
                # f.write(old )

                
            




