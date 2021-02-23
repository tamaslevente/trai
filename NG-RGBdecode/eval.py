# from dataset.dataloader import DepthDataset
#from utils.net_utils import adjust_learning_rate
from collections import Counter
from constants import *
from dataset.nyuv2_dataset import NYUv2Dataset
from imageio import imread
from model_fpn import I2D
from pathlib import Path
from PIL import Image
from scipy import misc
from threading import Thread
from torch.autograd import Variable
from torch.utils.data.sampler import Sampler
from torchvision import transforms
from torchvision.transforms import Resize, Compose, ToPILImage, ToTensor, RandomHorizontalFlip, CenterCrop, ColorJitter
from torchvision.utils import save_image
import argparse, time
import matplotlib, cv2
import matplotlib.pyplot as plt
import numpy as np
import os, sys
import random
import scipy.ndimage as ndimage
import timeit
import torch, time, os
import torch.nn as nn
import torch.nn.functional as F
import torch.utils.data as data
import cv2
matplotlib.use('Agg')




def parse_args():
    """
    Parse input arguments
    """
    parser = argparse.ArgumentParser(description='Single image depth estimation')
    parser.add_argument('--dataset', dest='dataset',
                      help='training dataset',
                      default='nyuv2', type=str)
    parser.add_argument('--epochs', dest='max_epochs',
                      help='number of epochs to train',
                      default=10, type=int)
    parser.add_argument('--cuda', dest='cuda',
                      help='whether use CUDA',
                      default=True,
                      action='store_true')
    parser.add_argument('--bs', dest='bs',
                      help='batch_size',
                      default=1, type=int)
    parser.add_argument('--num_workers', dest='num_workers',
                      help='num_workers',
                      default=1, type=int)
    parser.add_argument('--disp_interval', dest='disp_interval',
                      help='display interval',
                      default=10, type=int)
    parser.add_argument('--output_dir', dest='output_dir',
                      help='output directory',
                      default='saved_models', type=str)
    parser.add_argument('--input_image_path', dest='input_image_path',
                      help='path to a single input image for evaluation',
                      default='/media/cuda/ssd/datasets/naurgb/nyu_v2/depth3/train/', type=str)
    parser.add_argument('--eval_folder', dest='efolder',
                      help='evaluate only one image or the whole folder',
                      default=True)

# config optimization
    parser.add_argument('--o', dest='optimizer',
                      help='training optimizer',
                      default="sgd", type=str)
    parser.add_argument('--lr', dest='lr',
                      help='starting learning rate',
                      default=1e-3, type=float)
    parser.add_argument('--lr_decay_step', dest='lr_decay_step',
                      help='step to do learning rate decay, unit is epoch',
                      default=5, type=int)
    parser.add_argument('--lr_decay_gamma', dest='lr_decay_gamma',
                      help='learning rate decay ratio',
                      default=0.1, type=float)

# set training session
    parser.add_argument('--s', dest='session',
                      help='training session',
                      default=1, type=int)
    parser.add_argument('--eval_epoch', dest='eval_epoch',
                      help='number of epoch to evaluate',
                      default=2, type=int)

# resume trained model
    parser.add_argument('--r', dest='resume',
                      help='resume checkpoint or not',
                      default=True, type=bool)
    parser.add_argument('--start_at', dest='start_epoch',
                      help='epoch to start with',
                      default=0, type=int)
    parser.add_argument('--checksession', dest='checksession',
                      help='checksession to load model',
                      default=1, type=int)
    parser.add_argument('--checkepoch', dest='checkepoch',
                      help='checkepoch to load model',
                      default=99, type=int)
    parser.add_argument('--checkpoint', dest='checkpoint',
                      help='checkpoint to load model',
                      default=0, type=int)

# training parameters
    parser.add_argument('--gamma_sup', dest='gamma_sup',
                      help='factor of supervised loss',
                      default=1., type=float)
    parser.add_argument('--gamma_unsup', dest='gamma_unsup',
                      help='factor of unsupervised loss',
                      default=1., type=float)
    parser.add_argument('--gamma_reg', dest='gamma_reg',
                      help='factor of regularization loss',
                      default=10., type=float)

    args = parser.parse_args()
    return args

def resize_tensor(img, coords):
    return nn.functional.grid_sample(img, coords, mode='bilinear', padding_mode='zeros')


    
class sampler(Sampler):
  def __init__(self, train_size, batch_size):
    self.num_data = train_size
    self.num_per_batch = int(train_size / batch_size)
    self.batch_size = batch_size
    self.range = torch.arange(0,batch_size).view(1, batch_size).long()
    self.leftover_flag = False
    if train_size % batch_size:
      self.leftover = torch.arange(self.num_per_batch*batch_size, train_size).long()
      self.leftover_flag = True

  def __iter__(self):
    rand_num = torch.randperm(self.num_per_batch).view(-1,1) * self.batch_size
    self.rand_num = rand_num.expand(self.num_per_batch, self.batch_size) + self.range

    self.rand_num_view = self.rand_num.view(-1)

    if self.leftover_flag:
      self.rand_num_view = torch.cat((self.rand_num_view, self.leftover),0)

    return iter(self.rand_num_view)

  def __len__(self):
    return self.num_data

def collate_fn(data):
    imgs, depths = zip(*data)
    B = len(imgs)
    im_batch = torch.ones((B,3,376,1242))
    d_batch = torch.ones((B,1,376,1242))
    for ind in range(B):
        im, depth = imgs[ind], depths[ind]
        im_batch[ind, :, -im.shape[1]:, :im.shape[2]] = im
        d_batch[ind, :, -depth.shape[1]:, :depth.shape[2]] = depth
    return im_batch, d_batch

if __name__ == '__main__':

    args = parse_args()

    if torch.cuda.is_available() and not args.cuda:
        print("WARNING: You might want to run with --cuda")

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    # network initialization
    print('Initializing model...')
    i2d = I2D(fixed_feature_weights=False)
    if args.cuda:
        i2d = i2d.cuda()
        
    print('Done!')
    
    
    load_name = os.path.join(args.output_dir,
        'i2d_1_{}.pth'.format(args.checkepoch))
    print("loading checkpoint %s" % (load_name))
    state = i2d.state_dict()
    checkpoint = torch.load(load_name)
    args.start_epoch = checkpoint['epoch']
    checkpoint = {k: v for k, v in checkpoint['model'].items() if k in state}
    state.update(checkpoint)
    i2d.load_state_dict(state)
#         optimizer.load_state_dict(checkpoint['optimizer'])
#         lr = optimizer.param_groups[0]['lr']
    if 'pooling_mode' in checkpoint.keys():
        POOLING_MODE = checkpoint['pooling_mode']
    print("loaded checkpoint %s" % (load_name))
    del checkpoint
    torch.cuda.empty_cache()

    i2d.eval()

    img = Variable(torch.FloatTensor(1), volatile=True)

    print('evaluating...')
    height=480
    width=640
    if not args.efolder:
        rgb=Image.open(args.input_image_path)
        rgb2=Compose([Resize((height,width)), ToTensor()])(rgb)
        img = rgb2.unsqueeze_(0)
        z_fake = i2d(img.cuda())
        save_path=args.input_image_path[:-4]
        save_image(z_fake[0], save_path +"_pred"+'.png')
    else:
        start = timeit.default_timer()
        dlist=os.listdir(args.input_image_path)
        dlist.sort()
        images=[]
        for filename in dlist:
            if filename.endswith(".jpg") or filename.endswith(".png"):
                #print(os.path.join(directory, filename))
                images.append(filename)
            else:
                continue
        for i in range(len(images)):
            path=args.input_image_path+images[i]
            print("Predicting for:"+images[i])
            # rgb=Image.open(path)
            rgb = cv2.imread(path,cv2.IMREAD_UNCHANGED )
            # rgb2=Compose([Resize((360,640)), ToTensor()])(rgb)
            rgb2 = np.moveaxis(cv2.resize(rgb,(width,height)).astype(np.float32),-1,0)
            # img = rgb2.unsqueeze(0)
            img = torch.from_numpy(rgb2).float().unsqueeze(0)
            z_fake = i2d(img.cuda())
            # print("z_fake:"+str(torch.min(z_fake))+", "+str(torch.max(z_fake))+", "+str(torch.mean(z_fake)))
            zfv=z_fake*2-1
            z_fake_norm=zfv.pow(2).sum(dim=1).pow(0.5).unsqueeze(1)
            zfv=zfv/z_fake_norm
            z_fake=(zfv+1)/2
            save_path=path[:-4]
            save_image(z_fake[0], save_path +"_pred"+'.png')
            # cv2.imwrite(save_path +"_pred"+'.png',z_fake[0])
        stop = timeit.default_timer()
        print('Predicting '+str(len(images))+' images took ', stop - start)  
