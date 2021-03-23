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
    for i in range(num_eval_steps):

        if (i%10==0):
            print("New Point Cloud")

        print('step ' + str(i))
        
        ids, inputs, npts, gt, view_state, eval_value = next(test_gen)
        feed_dict = {inputs_pl: inputs, npts_pl: npts, gt_pl: gt, view_state_pl:view_state, 
            eval_value_pl:eval_value[:, :, :1], is_training_pl: False}
        start = time.time()
        test_loss, test_loss_eval, test_eval_value_pre = sess.run([model.loss, model.loss_eval, model.eval_value], feed_dict=feed_dict)

        #print(eval_value)
        #print(test_eval_value_pre)

        maximum1=-500
        maximum2=-500

        pozitie1=-1
        pozitie2=-1
        
        
        for k in range(args.views):
            if eval_value[0,k,0]>maximum1:
                pozitie1=k
                maximum1=eval_value[0,k,0]

        for k in range(args.views):
            if test_eval_value_pre[0,k,0]>maximum2:
                pozitie2=k
                maximum2=test_eval_value_pre[0,k,0]
        

        print("Predicted position:",pozitie2," ","Actual position:",pozitie1)
        

        
        
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
    sess.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--lmdb_test', default='/home/cuda/Alex/PC-NBV/data/train.lmdb')
    parser.add_argument('--model_type', default='pc-nbv')
    parser.add_argument('--checkpoint', default='/home/cuda/Alex/PC-NBV/log/7_11_1/model-400000')
    parser.add_argument('--batch_size', type=int, default=1)
    parser.add_argument('--num_input_points', type=int, default=512)
    parser.add_argument('--num_gt_points', type=int, default=1024)
    parser.add_argument('--views', type=int, default=33)
    parser.add_argument('--gpu', default='0')  #original default='2'

    args = parser.parse_args()

    train(args)