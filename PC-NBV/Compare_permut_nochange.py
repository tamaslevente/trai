import os
import numpy as np
import tensorflow as tf
import scipy.io as sio
from open3d import *
import random
from tf_ops.nn_distance import tf_nndistance 
import time
import pdb
from termcolor import colored

if __name__ == '__main__':

    
    
    # view num
    view_num = 16
    nr_of_models =122

    CRED = '\033[91m'
    CEND = '\033[0m'

    CGREEN  = '\33[32m'

    CYELLOW = '\33[33m'


    


    models_nochange_path ='/home/cuda/Alex/trai/PC-NBV/Archive/Archive_tests/Permuted_vs_nochange_122/Test_viewstate_model_nochange.txt'
    with open(os.path.join(models_nochange_path)) as file:
        model_id_list_nochange = [line.strip() for line in file]

    models_permuted_path ='/home/cuda/Alex/trai/PC-NBV/Archive/Archive_tests/Permuted_vs_nochange_122/Test_viewstate_model_permuted.txt'
    with open(os.path.join(models_permuted_path)) as file:
        model_id_list_permuted = [line.strip() for line in file]

    data_nochange_path='/home/cuda/Alex/trai/PC-NBV/Archive/Archive_tests/Permuted_vs_nochange_122/Test_viewstate_nochange.txt'
    data_nochange=np.loadtxt(data_nochange_path)
    
    data_permuted_path='/home/cuda/Alex/trai/PC-NBV/Archive/Archive_tests/Permuted_vs_nochange_122/Test_viewstate_permuted.txt'
    data_permuted=np.loadtxt(data_permuted_path)

    for i in range(view_num*nr_of_models):
        if(model_id_list_nochange[i]==model_id_list_permuted[i]):
            

            initial_pos_nochange=int(data_nochange[i][0])
            predicted_nochange=int(data_nochange[i][1])
            predicted_nochange_permuted = ((predicted_nochange - initial_pos_nochange) + 16 ) %16
            predicted_permuted=int(data_permuted[i][1])

            
            

            coverage_predicted_nochange=data_nochange[i][3]
            coverage_predicted_permuted=data_permuted[i][3]
            difference_predicted=coverage_predicted_nochange-coverage_predicted_permuted
            

            greedy_nochange=int(data_nochange[i][2])
            greedy_nochange_permuted = ((greedy_nochange - initial_pos_nochange) + 16 ) %16

            greedy_permuted=int(data_permuted[i][2])


            
            coverage_greedy_nochange=data_nochange[i][4]
            coverage_greedy_permuted=data_permuted[i][4]

            coverage_difference_final_nochange=coverage_greedy_nochange-coverage_predicted_nochange
            coverage_difference_final_permuted=coverage_greedy_permuted-coverage_predicted_permuted

            
            print(i)
            print("Initial position:"+ str(initial_pos_nochange))
            print("Predicted nochange:"+ str(predicted_nochange))

            if(predicted_nochange_permuted==predicted_permuted):
                print(CGREEN+"Predicted nochange permuted:"+ str(predicted_nochange_permuted)+" Predicted permuted:"+ str(predicted_permuted)+CEND)
            else:
                print("Predicted nochange permuted:"+ str(predicted_nochange_permuted)+" Predicted permuted:"+ str(predicted_permuted))
            
            print("Greedy nochange:"+ str(greedy_nochange))
            print("Greedy nochange permuted:"+ str(greedy_nochange_permuted)+" Greedy permuted:"+ str(greedy_permuted))
      
            print("Coverage nochange:"+str(coverage_predicted_nochange)+" Coverage_permuted:"+str(coverage_predicted_permuted)+ " Coverage difference:"+str(difference_predicted))

            print("cov_dif(greedy,nochange): "+str(coverage_difference_final_nochange) + " cov_dif(greedy,permuted): "+str(coverage_difference_final_permuted))
            