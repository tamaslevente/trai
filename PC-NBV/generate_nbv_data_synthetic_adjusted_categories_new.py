import os
import numpy as np
import tensorflow as tf
import scipy.io as sio
from open3d import *
import random
from tf_ops.nn_distance import tf_nndistance 
import time
import pdb

if __name__ == '__main__':

    os.environ['CUDA_VISIBLE_DEVICES'] = "0"
    
    # view num
    #view_num = 33
    view_num = 4
    nr_views_choose=1

    # path
    data_type = 'train/'
    ShapeNetv1_dir = '/home/cuda/Alex/trai/PC-NBV/data/Data_external/Voxel/'    
    pc_dir = "/home/cuda/Alex/trai/PC-NBV/Output_model_blender/" + data_type + "/pcd"
    save_dir = "/home/cuda/Alex/trai/PC-NBV/data/Data_external/NBV_data/shapenet_33_views_640x480/"+ data_type
    model_dir = '/home/cuda/Alex/trai/PC-NBV/data/Data_external/Voxel/' + data_type

    
   
    # # for calculating surface coverage and register
    part_tensor = tf.placeholder(tf.float32, (1, None, 3))
    gt_tensor = tf.placeholder(tf.float32, (1, None, 3))
    sess = tf.Session()
    dist1, _, dist2, _ = tf_nndistance.nn_distance(part_tensor, gt_tensor)
    
    class_list = os.listdir(model_dir)

    f=open('generate_nbv.log', 'w+')

    for class_id in class_list:

        model_list = os.listdir(os.path.join(ShapeNetv1_dir, data_type, class_id))

        for model in model_list:
            save_model_path = os.path.join(save_dir, model)
            if os.path.exists(save_model_path):
                print("skip " + save_model_path)
                continue

            # gt point cloud
            #gt_points = sio.loadmat(os.path.join(ShapeNetv1_dir, data_type, class_id, model, 'model.mat'))

            gt_points = sio.loadmat('/home/cuda/Alex/trai/PC-NBV/Shapenet_v1/Archive/cub/cub/cub/model.mat')


            gt_points = np.array(gt_points['points'])
            if not os.path.exists(os.path.join(save_dir, model)):
                os.makedirs(os.path.join(save_dir, model))

            np.savetxt(os.path.join(save_dir, model, "gt.xyz"), gt_points)    

            # every view's partial point cloud
            part_points_list = []
            
            for i in range(view_num):
                t=i+1
                pcd_path = os.path.join(model_dir,class_id, model, str(t) + ".pcd")
                if os.path.exists(pcd_path):
                    print("Pointcloud found")
                    cur_pc = open3d.io.read_point_cloud(pcd_path)
                    cur_points = np.asarray(cur_pc.points)  

                    print(cur_points)
                else:
                    cur_points = np.zeros((1,3))

                part_points_list.append(cur_points)


            # reconstruct from different views 1 times
            selected_init_view = []
            for ex_index in range(1): 
            #for ex_index in range(16):  

                start = time.time() 

                cur_ex_dir = os.path.join(save_dir, model, str(ex_index))
                if not os.path.exists(cur_ex_dir):
                    os.makedirs(cur_ex_dir) 

                # init view state
                view_state = np.zeros(view_num, dtype=np.int) # 0 unselected, 1 selected, 2 cur

                
                # init start view
                while (True):
                   # cur_view = random.randint(0, view_num - 1)
                    cur_view = random.randint(0, nr_views_choose-1)
                    if not cur_view in selected_init_view:
                        selected_init_view.append(cur_view)
                        break   
                print(str(class_id))

                print(str(view_state))

                if ( (("pos1") in class_id) or (("Pos1") in class_id) ):
                    print("Position is pos_1")
                    view_state[0]=1

                if ( (("pos2") in class_id) or (("Pos2") in class_id) ):
                    print("Position is pos_2")
                    view_state[1]=1
                if ( (("pos3") in class_id) or (("Pos3") in class_id) ):
                    print("Position is pos_3")
                    view_state[2]=1
                if ( (("pos4") in class_id) or (("Pos4") in class_id) ):
                    print("Position is pos_4")
                    view_state[3]=1

                # # view_state[cur_view] = 1
                # if (class_id.find("Top_050")):
                #     print("Top_050")
                #     view_state[0]=1
                # elif (class_id.find("Top_100")):
                #     view_state[1]=1
                # elif (class_id.find('Sameheight_050')):
                #     view_state[2]=1
                # elif (class_id.find('Sameheight_100')):
                #     print("Sameheight_100")
                #     view_state[3]=1

                print(str(class_id)+" "+str(view_state))

                


                #view_state[0]=1

                acc_pc_points = part_points_list[cur_view]  

                # # accumulate points coverage
                # batch_acc = acc_pc_points[np.newaxis, :, :]
                # batch_gt = gt_points[np.newaxis, :, :]

                # dist2_new = sess.run(dist2, feed_dict={part_tensor:batch_acc, gt_tensor:batch_gt})      
                # dis_flag_new = dist2_new < 0.00005
                # cover_sum = np.sum(dis_flag_new == True)
                # cur_cov = cover_sum / dis_flag_new.shape[1]

                

                # max scan 10 times
                for scan_index in range(1):    

                    #print("coverage:" + str(cur_cov) + " in scan round " + str(scan_index)) 

                    #f.write("coverage:" + str(cur_cov) + " in scan round " + str(scan_index) +'\n')

                    np.save(os.path.join(cur_ex_dir, str(scan_index) + "_viewstate.npy") ,view_state)
                    np.save(os.path.join(cur_ex_dir, str(scan_index) + "_acc_pc.npy"), acc_pc_points)
                    # np.savetxt(os.path.join(cur_ex_dir, str(scan_index) + "_acc_pc.xyz"), acc_pc_points)    

                    target_value = np.zeros((view_num, 1)) # surface coverage, register coverage, moving cost for each view         

                    max_view_index = 0
                    max_view_cov = 0
                    max_new_pc = np.zeros((1,3))

                    # # accumulate points coverage
                    batch_acc = acc_pc_points[np.newaxis, :, :]
                    batch_gt = gt_points[np.newaxis, :, :]

                    if ( (('Box2') in class_id) or (('Box_2') in class_id) ):
                            target_value[0, 0]=0.2
                            target_value[1, 0]=0.2
                            target_value[2, 0]=0.2
                            target_value[3, 0]=0.8
                    
                    if ( (('Box3') in class_id) or (('Box_3') in class_id) ):
                                target_value[0, 0]=0.2
                                target_value[1, 0]=0.2
                                target_value[2, 0]=0.2
                                target_value[3, 0]=0.8

                    if ( (('Box4') in class_id) or (('Box_4') in class_id) ):
                                target_value[0, 0]=0.8
                                target_value[1, 0]=0.2
                                target_value[2, 0]=0.2
                                target_value[3, 0]=0.2

                    if ( (('Box5') in class_id) or (('Box_5') in class_id) ):
                                target_value[0, 0]=0.2
                                target_value[1, 0]=0.2
                                target_value[2, 0]=0.8
                                target_value[3, 0]=0.2

                    if ( (('Box6') in class_id) or (('Box_6') in class_id) ):
                                target_value[0, 0]=0.2
                                target_value[1, 0]=0.2
                                target_value[2, 0]=0.8
                                target_value[3, 0]=0.2

                    if ( (('Box7') in class_id) or (('Box_7') in class_id) ):
                                target_value[0, 0]=0.2
                                target_value[1, 0]=0.8
                                target_value[2, 0]=0.2
                                target_value[3, 0]=0.2

                    if ( (('Box9') in class_id) or (('Box_9') in class_id) ):
                                target_value[0, 0]=0.2
                                target_value[1, 0]=0.8
                                target_value[2, 0]=0.2
                                target_value[3, 0]=0.2
                       
                        
                      
                    
                    # evaluate all the views
                    #for i in range(view_num): 

                         
                         
                        # current evaluate view
                        # batch_part_cur = part_points_list[i][np.newaxis, :, :]  

                        # # new pc
                        # dist1_new = sess.run(dist1, feed_dict={part_tensor:batch_part_cur, gt_tensor:batch_acc})
                        # dis_flag_new = dist1_new < 0.00005  

                        # pc_register = batch_part_cur[dis_flag_new]
                        # pc_new = batch_part_cur[~dis_flag_new]

                        # batch_new = pc_new[np.newaxis, :, :]    

                        # # test new coverage
                        # if batch_new.shape[1] != 0:
                        #     dist2_new = sess.run(dist2, feed_dict={part_tensor:batch_new, gt_tensor:batch_gt})      

                        #     dis_flag_new = dist2_new < 0.00005
                        #     cover_sum = np.sum(dis_flag_new == True)
                        #     cover_new = cover_sum / dis_flag_new.shape[1]
                        # else:
                        #     cover_new = 0   

                        # target_value[i, 0] = cover_new

                        # if ( target_value[i, 0] > max_view_cov ):
                        #     max_view_index = i
                        #     max_view_cov = target_value[i, 0]
                        #     max_new_pc = pc_new 


                    print(str(target_value))

                    np.save(os.path.join(cur_ex_dir, str(scan_index) + "_target_value.npy"), target_value)  

                    # print("choose view:" + str(max_view_index) + " add coverage:" + str(max_view_cov)) 

                    # f.write("choose view:" + str(max_view_index) + " add coverage:" + str(max_view_cov) +'\n')  

                    # cur_view = max_view_index
                    # cur_cov = cur_cov + target_value[max_view_index, 0]
                    # view_state[cur_view] = 1
                    # acc_pc_points = np.append(acc_pc_points, max_new_pc, axis=0)    

                    print('scan %s done, time=%.4f sec' % (scan_index, time.time() - start))

                    f.write('scan %s done, time=%.4f sec' % (scan_index, time.time() - start) +'\n')


            


