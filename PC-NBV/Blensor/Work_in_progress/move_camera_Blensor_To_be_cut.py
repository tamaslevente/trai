import argparse
import os
from io_util import read_pcd
from tensorpack import DataFlow, dataflow
from open3d import *
import numpy as np
import scipy.io as sio
import pdb
import datetime
import importlib
import models
import tensorflow as tf
import time
from data_util_nbv import lmdb_dataflow, get_queued_data
from termcolor import colored
import pdb
import csv
from scipy import stats



class pcd_df(DataFlow):
    def __init__(self, class_list, ex_times, num_scans, NBV_dir, gt_dir, data_type):
        self.class_list = class_list
        self.num_scans = num_scans
        self.ex_times = ex_times
        self.NBV_dir = NBV_dir
        self.gt_dir = gt_dir
        self.data_type = data_type

    def size(self):
        if self.data_type == 'valid':
            return 4000
        elif self.data_type == 'train':
            return 40000
        elif self.data_type == 'test':
            return 4000
        elif self.data_type == 'test_novel':
            return 4000
        elif self.data_type == 'cub':
            return 4000

    def get_data(self):

        pcd_path = '/home/cuda/Alex/trai/PC-NBV/test_lmdb_blensor/1.pcd'

        cur_pc = open3d.io.read_point_cloud(pcd_path)
        cur_points = np.asarray(cur_pc.points)  

        acc_pc_points=cur_points
        
        model_id='model_test'
        gt_points = sio.loadmat('/home/cuda/Alex/trai/PC-NBV/test_lmdb_blensor/model.mat')
        gt_pc = np.array(gt_points['points']) # shape (16384, 3)
        view_state = np.load('/home/cuda/Alex/trai/PC-NBV/test_lmdb_blensor/0_viewstate_permuted.npy') # shape (33) , 33 is view number
        accumulate_pointcloud = acc_pc_points
        target_value = np.load('/home/cuda/Alex/trai/PC-NBV/test_lmdb_blensor/0_target_value_permuted.npy') # shape (33, 1), 33 is view number
        yield model_id, accumulate_pointcloud, gt_pc, view_state, target_value


def train(args):
    os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu

    is_training_pl = tf.placeholder(tf.bool, shape=(), name='is_training')
    global_step = tf.Variable(0, trainable=False, name='global_step')
    inputs_pl = tf.placeholder(tf.float32, (1, None, 3), 'inputs') # input point cloud
    npts_pl = tf.placeholder(tf.int32, (args.batch_size,), 'num_points') 
    gt_pl = tf.placeholder(tf.float32, (args.batch_size, args.num_gt_points, 3), 'ground_truths') # ground truth
    view_state_pl = tf.placeholder(tf.float32, (args.batch_size, args.views), 'view_state') # view space selected state
    eval_value_pl = tf.placeholder(tf.float32, (args.batch_size, args.views, 1), 'eval_value') # surface cov, 

    model_module = importlib.import_module('.%s' % args.model_type, 'models')
    model = model_module.Model(inputs_pl, npts_pl, gt_pl, view_state_pl, eval_value_pl, is_training = is_training_pl)

    df_test, num_test = lmdb_dataflow(
        args.lmdb_test, args.batch_size, args.num_input_points, args.num_gt_points, is_training=False)
    test_gen = df_test.get_data()

    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    config.allow_soft_placement = True
    sess = tf.Session(config=config)
    saver = tf.train.Saver(max_to_keep=100)

    saver.restore(sess, args.checkpoint)

    total_time = 0
    train_start = time.time()

    print(colored('Testing...', 'grey', 'on_green'))
    num_eval_steps = num_test // args.batch_size
    test_total_loss = 0
    test_total_loss_eval = 0
    test_total_time = 0
    test_total_spearmanr = 0
    sess.run(tf.local_variables_initializer())

   

    

    pozitie1_global=-1

    CRED = '\033[91m'
    CEND = '\033[0m'

    CGREEN  = '\33[32m'

    CYELLOW = '\33[33m'

    threshold_coverage_difference=0.05
        

    for i in range(num_eval_steps):

        # if (i%15==0):
        #     print("New Point Cloud")

        print('step ' + str(i))
        
        ids, inputs, npts, gt, view_state, eval_value = next(test_gen)
        feed_dict = {inputs_pl: inputs, npts_pl: npts, gt_pl: gt, view_state_pl:view_state, 
            eval_value_pl:eval_value[:, :, :1], is_training_pl: False}
        start = time.time()
        test_loss, test_loss_eval, test_eval_value_pre = sess.run([model.loss, model.loss_eval, model.eval_value], feed_dict=feed_dict)

        #print(eval_value)
        #print(test_eval_value_pre)
        # if (i==0):
        #     for m in range(args.views):
        #         if view_state[0,m]==1:
        #             pozitie_actuala=m
        # else:
        #         pozitie_actuala=pozitie_greedy



        for m in range(args.views):
                if view_state[0,m]==1:
                    pozitie_actuala=m

        maximum_predict=-500
        maximum_greedy=-500

        pozitie_greedy=-1
        pozitie_predict=-1
        
        
        for k in range(args.views):
            if eval_value[0,k,0]>maximum_greedy:
                pozitie_greedy=k
                maximum_greedy=eval_value[0,k,0]

        for k in range(args.views): 
            if test_eval_value_pre[0,k,0]>maximum_predict:
                pozitie_predict=k
                maximum_predict=test_eval_value_pre[0,k,0]

        pozitie1_global=pozitie_greedy
        
        
        
        nr_ok=0;

        # if(abs(maximum_predict-maximum_greedy)<threshold_coverage_difference):
        #     print(CGREEN+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Corect"+CEND)
        #     print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))
        # else:
        #     print(CRED+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Wrong"+CEND)
        #     print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))
        

        if(pozitie_predict==pozitie_greedy):
            nr_ok=1
            print(CGREEN+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Corect"+CEND)
            print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))
        else:
            nr_ok=0
            print(CRED+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Wrong"+CEND)
            print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))

        # print("Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",pozitie_predict," ","Greedy position:",pozitie_greedy)
        
        predict_coverage_real=eval_value[0,pozitie_predict,0]
        
        difference = maximum_greedy-predict_coverage_real
        

        aux=str(ids)
        better_id=aux.replace("['","")
        better_id_2=better_id.replace("']","")

        f=open('/home/cuda/Alex/trai/PC-NBV/data/Data_lmdb_Blensor/output.txt', 'w+')
        
        f.write(str(pozitie_predict))

        
        
        test_total_time += time.time() - start
        test_spearmanr_batch_total = 0
        for j in range(args.batch_size):
            test_spearmanr_batch_total += stats.spearmanr(eval_value[j, :, 0], test_eval_value_pre[j, :, 0])[0]
        test_spearmanr = test_spearmanr_batch_total / args.batch_size
        test_total_loss += test_loss
        test_total_loss_eval += test_loss_eval
        test_total_spearmanr += test_spearmanr

        
    #summary = sess.run(test_summary, feed_dict={is_training_pl: False})
    print(colored('loss %.8f loss_eval %.8f spearmanr %.8f - time per batch %.4f' %
                  (test_total_loss / num_eval_steps, test_total_loss_eval / num_eval_steps,
                     test_total_spearmanr / num_eval_steps, test_total_time / num_eval_steps),
                  'grey', 'on_green'))
    test_total_time = 0

    

    print('Total time', datetime.timedelta(seconds=time.time() - train_start))

     # need to change to make it general


    sess.close()

    


if __name__ == '__main__':

    # data_type = 'valid'
    # class_list_path = '/home/zengrui/IROS/pcn/data/ShapeNetv1/' + data_type + '_class.txt'
    # gt_dir = "/home/zengrui/IROS/pcn/data/ShapeNetv1/" + data_type
    # output_path = "data/" + data_type + ".lmdb"
    # NBV_dir = "/home/zengrui/IROS/pcn/NBV_data/shapenet_33_views"

    data_type = 'test'
    #class_list_path = '/home/cuda/Alex/trai/PC-NBV/Shapenet_v1/Synthetic_step_4/Classes/' + data_type + '/_class.txt'
    class_list_path = '/home/cuda/Alex/trai/PC-NBV/Shapenet_v1/Synthetic_step_4/Classes' +  '/_class_test.txt'
    gt_dir = "/home/cuda/Alex/trai/PC-NBV/Shapenet_v1/Synthetic_step_4/" + data_type
    output_path = "data/Data_lmdb_Blensor/" + data_type + ".lmdb"
    NBV_dir = "/home/cuda/Alex/trai/PC-NBV/NBV_data/shapenet_33_views_640x480/"+data_type

    


    #NBV_dir = "/home/cuda/Alex/trai/PC-NBV/NBV_data_no_target/shapenet_33_views_640x480/test"


    ex_times = 1
    num_scans = 1

    with open(os.path.join(class_list_path)) as file:
        class_list = [line.strip() for line in file]

    df = pcd_df(class_list, ex_times, num_scans, NBV_dir, gt_dir, data_type)
    if os.path.exists(output_path):
        os.system('rm %s' % output_path)

    dataflow.LMDBSerializer.save(df, output_path)

    parser = argparse.ArgumentParser()
    parser.add_argument('--lmdb_test', default='/home/cuda/Alex/trai/PC-NBV/data/Data_lmdb_Blensor/test.lmdb')
    parser.add_argument('--model_type', default='pc-nbv')
    parser.add_argument('--checkpoint', default='/home/cuda/Alex/trai/PC-NBV/log/New_test/model-400000')
    parser.add_argument('--batch_size', type=int, default=1)
    parser.add_argument('--num_input_points', type=int, default=512)
    parser.add_argument('--num_gt_points', type=int, default=1024)
    parser.add_argument('--views', type=int, default=4) # original 33
    parser.add_argument('--gpu', default='0')  #original default='2'

    args = parser.parse_args()

    train(args)