import argparse
import datetime
import importlib
import models
import os
import tensorflow as tf
import time
from data_util_nbv import lmdb_dataflow, get_queued_data
from termcolor import colored
import pdb
from tensorpack import dataflow
from scipy import stats
import csv
import numpy as np
from numpy import dot
from numpy.linalg import norm


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

    f=open('Test_viewstate.txt', 'w+')
    f2=open('Test_viewstate_model.txt', 'w+')

    

    pozitie1_global=-1

    CRED = '\033[91m'
    CEND = '\033[0m'

    CGREEN  = '\33[32m'

    CYELLOW = '\33[33m'

    threshold_coverage_difference=0.05

    nr_corect_pcd=0   

    for i in range(num_eval_steps):

        # if (i%15==0):
        #     print("New Point Cloud")

        print('step ' + str(i))
        
        ids, inputs, npts, gt, view_state, eval_value = next(test_gen)
        feed_dict = {inputs_pl: inputs, npts_pl: npts, gt_pl: gt, view_state_pl:view_state, 
            eval_value_pl:eval_value[:, :, :1], is_training_pl: False}
        start = time.time()
        test_loss, test_loss_eval, test_eval_value_pre = sess.run([model.loss, model.loss_eval, model.eval_value], feed_dict=feed_dict)

        
        # print("Scores by greedy"+'\n')
        # print(np.shape(eval_value))
        #print(eval_value)
        
        # print("Scores by network")
        # print(np.shape(test_eval_value_pre))
        #print(test_eval_value_pre)

        # scores_final=np.zeros((40, 2))

        # scores_final[:,0]=eval_value[0,:,0]
        # scores_final[:,1]=test_eval_value_pre[0,:,0]


        # print(scores_final)
        


        # cos_sim_total = dot(eval_value[0,:,0], test_eval_value_pre[0,:,0])/(norm(eval_value[0,:,0])*norm(test_eval_value_pre[0,:,0]))

        # print("Global similarity")
        # print(cos_sim_total)

        # cos_sim_1 = dot(eval_value[0,0:9,0], test_eval_value_pre[0,0:9,0])/(norm(eval_value[0,0:9,0])*norm(test_eval_value_pre[0,0:9,0]))
        # cos_sim_2 = dot(eval_value[0,10:19,0], test_eval_value_pre[0,10:19,0])/(norm(eval_value[0,10:19,0])*norm(test_eval_value_pre[0,10:19,0]))
        # cos_sim_3 = dot(eval_value[0,20:29,0], test_eval_value_pre[0,20:29,0])/(norm(eval_value[0,20:29,0])*norm(test_eval_value_pre[0,20:29,0]))
        # cos_sim_4 = dot(eval_value[0,30:39,0], test_eval_value_pre[0,30:39,0])/(norm(eval_value[0,30:39,0])*norm(test_eval_value_pre[0,30:39,0]))

        # print("Similarity part 1")
        # print(cos_sim_1)
        # print("Similarity part 2")
        # print(cos_sim_2)
        # print("Similarity part 3")
        # print(cos_sim_3)
        # print("Similarity part 4")
        # print(cos_sim_4)

       

        
        maximum_predict=-500
        maximum_greedy=-500

        pozitie_greedy=-1
        pozitie_predict=-1
        
        
        # for k in range(4):
        #     if eval_value[0,10*k,0]>maximum_greedy:
        #         pozitie_greedy=k
        #         maximum_greedy=eval_value[0,10*k,0]

        # for k in range(4): 
        #     if test_eval_value_pre[0,10*k,0]>maximum_predict:
        #         pozitie_predict=k
        #         maximum_predict=test_eval_value_pre[0,10*k,0]

        for k in range(4):
            if eval_value[0,k,0]>maximum_greedy:
                pozitie_greedy=k
                maximum_greedy=eval_value[0,k,0]

        for k in range(4): 
            if test_eval_value_pre[0,k,0]>maximum_predict:
                pozitie_predict=k
                maximum_predict=test_eval_value_pre[0,k,0]

        if(pozitie_predict==pozitie_greedy):
            nr_corect_pcd=nr_corect_pcd+1
            print(CGREEN+str(ids)+" Right"+CEND)
        else:
            print(CRED+str(ids)+" Wrong"+CEND)
            




        #print(eval_value)
        #print(test_eval_value_pre)
        # if (i==0):
        #     for m in range(args.views):
        #         if view_state[0,m]==1:
        #             pozitie_actuala=m
        # else:
        #         pozitie_actuala=pozitie_greedy



        # for m in range(args.views):
        #         if view_state[0,m]==1:
        #             pozitie_actuala=m

        # maximum_predict=-500
        # maximum_greedy=-500

        # pozitie_greedy=-1
        # pozitie_predict=-1
        
        
        # for k in range(args.views):
        #     if eval_value[0,k,0]>maximum_greedy:
        #         pozitie_greedy=k
        #         maximum_greedy=eval_value[0,k,0]

        # for k in range(args.views): 
        #     if test_eval_value_pre[0,k,0]>maximum_predict:
        #         pozitie_predict=k
        #         maximum_predict=test_eval_value_pre[0,k,0]

        # pozitie1_global=pozitie_greedy
        
        
        
        # nr_ok=0;

        # if(abs(maximum_predict-maximum_greedy)<threshold_coverage_difference):
        #     print(CGREEN+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Corect"+CEND)
        #     print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))
        # else:
        #     print(CRED+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Wrong"+CEND)
        #     print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))
        

        # if(pozitie_predict==pozitie_greedy):
        #     nr_ok=1
        #     print(CGREEN+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Corect"+CEND)
        #     print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))
        #     nr_corect_pcd=nr_corect_pcd+1
        # else:
        #     nr_ok=0
        #     print(CRED+str(ids)+" Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",str(pozitie_predict)," ","Greedy position:",str(pozitie_greedy)+" Wrong"+CEND)
        #     print("Predicted coverage:"+str(maximum_predict)+" "+"Greedy coverage:"+str(maximum_greedy))

        # # print("Current Position:"+str(pozitie_actuala)+" "+str(view_state)+" "+" Predicted position:",pozitie_predict," ","Greedy position:",pozitie_greedy)
        
        # predict_coverage_real=eval_value[0,pozitie_predict,0]
        
        # difference = maximum_greedy-predict_coverage_real
        # f.write(str(pozitie_actuala)+" "+str(pozitie_predict)+" "+str(pozitie_greedy)+" "+str(predict_coverage_real)+" "+str(maximum_greedy)+" "+str(difference)+'\n')

        # aux=str(ids)
        # better_id=aux.replace("['","")
        # better_id_2=better_id.replace("']","")

        # f2.write(str(better_id_2)+'\n')

        

        
        
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

    print("Corect estimations: "+str(nr_corect_pcd)+"/"+str(num_eval_steps))
    # test_total_time = 0

    print('Total time', datetime.timedelta(seconds=time.time() - train_start))
    sess.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--lmdb_test', default='/home/cuda/Alex/trai/PC-NBV/data/Data_external/Ahriva/output.lmdb')
    parser.add_argument('--model_type', default='pc-nbv_2')
    parser.add_argument('--checkpoint', default='/home/cuda/Alex/trai/PC-NBV/data/Data_external/Ahriva/Network_model_histograms/model-230000')
    parser.add_argument('--batch_size', type=int, default=1)
    parser.add_argument('--num_input_points', type=int, default=512)
    parser.add_argument('--num_gt_points', type=int, default=1024)
    parser.add_argument('--views', type=int, default=4) # original 33
    parser.add_argument('--gpu', default='0')  #original default='2'

    args = parser.parse_args()

    train(args)