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
    view_num = 33


    # path
    data_type = 'train/'
    ShapeNetv1_dir = '/home/cuda/Alex/trai/PC-NBV/Shapenet_v1/'    
    pc_dir = "/home/cuda/Alex/trai/PC-NBV/Output_model_blender/" + data_type + "/pcd"
    save_dir = "/home/cuda/Alex/trai/PC-NBV/NBV_data/shapenet_33_views_640x480/train"
    model_dir = '/home/cuda/Alex/trai/PC-NBV/Shapenet_v1/' + data_type

    

    # for calculating surface coverage and register
    part_tensor = tf.placeholder(tf.float32, (1, None, 3))
    gt_tensor = tf.placeholder(tf.float32, (1, None, 3))
    sess = tf.Session()
    dist1, _, dist2, _ = tf_nndistance.nn_distance(part_tensor, gt_tensor)

    class_list = os.listdir(model_dir)

    f=open('View_coverage.log', 'w+')

    test_predicted_path ='/home/cuda/Alex/trai/PC-NBV/Test_viewstate.txt'

    test_viewstate=np.loadtxt(test_predicted_path)

    nr_elemente=0

    



    for class_id in class_list:

        model_list = os.listdir(os.path.join(ShapeNetv1_dir, data_type, class_id))

        for model in model_list:
            
            nr_elemente=nr_elemente+1

            # gt point cloud
            gt_points = sio.loadmat(os.path.join(ShapeNetv1_dir, data_type, class_id, model, 'model.mat'))
            gt_points = np.array(gt_points['points'])    

            # every view's partial point cloud
            part_points_list = []
            
            for i in range(view_num):
                pcd_path = os.path.join(pc_dir, model, str(i) + ".pcd")
                if os.path.exists(pcd_path):
                    cur_pc = open3d.io.read_point_cloud(pcd_path)
                    cur_points = np.asarray(cur_pc.points)  
                else:
                    cur_points = np.zeros((1,3))

                part_points_list.append(cur_points)

            # reconstruct from different views 1 times
            selected_init_view = []

            
            
            
            for i in range(10): 
            
                
                pozitie_actuala=int(test_viewstate[(nr_elemente-1)*10+i][0])
                
                
                view_state = np.zeros(view_num, dtype=np.int) # 0 unselected, 1 selected, 2 cur

                cur_view = pozitie_actuala
               
                view_state[cur_view] = 1

                if (i==0):
                    acc_pc_points = part_points_list[cur_view]  
                else:
                    acc_pc_points = np.append(acc_pc_points, part_points_list[cur_view], axis=0)

                # accumulate points coverage
                batch_acc = acc_pc_points[np.newaxis, :, :]
                batch_gt = gt_points[np.newaxis, :, :]

                dist2_new = sess.run(dist2, feed_dict={part_tensor:batch_acc, gt_tensor:batch_gt})      
                dis_flag_new = dist2_new < 0.00005
                cover_sum = np.sum(dis_flag_new == True)
                cur_cov = cover_sum / dis_flag_new.shape[1]

                print("Pozitie actuala:"+str(cur_view)+" coverage:" + str(cur_cov) + " in scan round " + str(i)) 

                f.write("Pozitie actuala:"+str(cur_view)+" coverage:" + str(cur_cov) + " in scan round " + str(i) +'\n')

                  

                    # # accumulate points coverage
                batch_acc = acc_pc_points[np.newaxis, :, :]
                batch_gt = gt_points[np.newaxis, :, :]
                    
                    # evaluate all the views
                       # trebuie schimbat ca sa selecteze predicted position

                pozitie_prezisa=int(test_viewstate[(nr_elemente-1)*10+i][1])

                        # current evaluate view
                batch_part_cur = part_points_list[pozitie_prezisa][np.newaxis, :, :]  

                #         # new pc
                dist1_new = sess.run(dist1, feed_dict={part_tensor:batch_part_cur, gt_tensor:batch_acc})
                dis_flag_new = dist1_new < 0.00005  

                pc_register = batch_part_cur[dis_flag_new]
                pc_new = batch_part_cur[~dis_flag_new]

               
                # pc_new=acc_pc_points
                # pc_new=np.append(pc_new, part_points_list[pozitie_prezisa], axis=0)
                batch_new = pc_new[np.newaxis, :, :]    

                        # test new coverage
                if batch_new.shape[1] != 0:
                    dist2_new = sess.run(dist2, feed_dict={part_tensor:batch_new, gt_tensor:batch_gt})      

                    dis_flag_new = dist2_new < 0.00005
                    cover_sum = np.sum(dis_flag_new == True)
                    cover_new = cover_sum / dis_flag_new.shape[1]
                else:
                    cover_new = 0   

                predict_coverage=cur_cov+cover_new

                print("Pozitie next predict:"+str(pozitie_prezisa)+" Coverage predicted:"+str(predict_coverage)+"in scan round " + str(i))

                f.write("Pozitie next predict:"+str(pozitie_prezisa)+" Coverage predicted:" + str(predict_coverage) + " in scan round " + str(i) +'\n')


                pozitie_greedy=int(test_viewstate[(nr_elemente-1)*10+i][2])


                batch_part_cur = part_points_list[pozitie_greedy][np.newaxis, :, :]  

                #         # new pc
                dist1_new = sess.run(dist1, feed_dict={part_tensor:batch_part_cur, gt_tensor:batch_acc})
                dis_flag_new = dist1_new < 0.00005  

                pc_register = batch_part_cur[dis_flag_new]
                pc_new = batch_part_cur[~dis_flag_new]

                # pc_new=acc_pc_points
                # pc_new=np.append(pc_new, part_points_list[pozitie_greedy], axis=0)

                batch_new = pc_new[np.newaxis, :, :]    

                        # test new coverage
                if batch_new.shape[1] != 0:
                    dist2_new = sess.run(dist2, feed_dict={part_tensor:batch_new, gt_tensor:batch_gt})      

                    dis_flag_new = dist2_new < 0.00005
                    cover_sum = np.sum(dis_flag_new == True)
                    cover_new2 = cover_sum / dis_flag_new.shape[1]
                else:
                    cover_new2 = 0   

                greedy_coverage=cur_cov+cover_new2

                print("Pozitie next greedy:"+str(pozitie_greedy)+" Coverage greedy:"+str(greedy_coverage)+"in scan round " + str(i)+'\n')

                f.write("Pozitie next greedy:"+str(pozitie_greedy)+" Coverage greedy:" + str(greedy_coverage) + " in scan round " + str(i) +'\n')

                        

                       

                       

                   

            


