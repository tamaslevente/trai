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
    view_num = 16

    # path
    data_type = 'test/'

    NBV_file= "/home/cuda/Alex/trai/PC-NBV/NBV_data/shapenet_33_views_640x480/"
    
    save_dir = "/home/cuda/Alex/trai/PC-NBV/NBV_data/shapenet_33_views_640x480/"+ data_type
    

    save_type='test2_permuted/'

    save_dir_final= NBV_file+save_type

    if not os.path.exists(save_dir_final):
        os.makedirs(save_dir_final)

    # for calculating surface coverage and register
    

    model_list = os.listdir(os.path.join(save_dir))

    for model in model_list:

        if not os.path.exists(os.path.join(save_dir_final, model)):
                    os.makedirs(os.path.join(save_dir_final, model))
                
       
        for i in range(view_num):
                if not os.path.exists(os.path.join(save_dir_final, model,str(i))):
                    os.makedirs(os.path.join(save_dir_final, model,str(i)))


                score_path = os.path.join(save_dir,model, str(i) , "0_target_value.npy")
                state_path = os.path.join(save_dir,model, str(i) , "0_viewstate.npy")

                Viewstate=np.load(state_path, mmap_mode='r')

                pos=0

                for j in range(view_num):
                    if(Viewstate[j]==1):
                        pos=j
                print(pos)

                View_scores = np.load(score_path, mmap_mode='r')
                View_scores_permuted = (np.roll(View_scores, -pos))

                

                np.savetxt(os.path.join(save_dir_final, model,str(i),"0_target_value_permuted.npy"),View_scores_permuted)    
              
         
                

            


