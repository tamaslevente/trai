import torch
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import random
random.seed(10)

from zmq import device
from constants import *
from model_fpn_curv_grad import I2D
import argparse
import time
# from utils.net_utils import adjust_learning_rate
from torch.autograd import Variable
# from dataset.dataloader import DepthDataset
# from dataset.nyuv2_dataset import NYUv2Dataset
from torchvision.utils import save_image
from dataset.nyuv2_dataset import MyCustomDataset
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from collections import Counter
import matplotlib
import open3d as o3d
import math

matplotlib.use('Agg')

from torch.utils.tensorboard import SummaryWriter
writer = SummaryWriter()


class DDDDepthDiff(nn.Module):
    def __init__(self):
        super(DDDDepthDiff, self).__init__()

    def forward(self, fake, real,epoch,show_image):
        if not fake.shape == real.shape:
            _, _, H, W = real.shape
            fake = F.interpolate(fake, size=(H, W), mode='bilinear')
        eps = 1e-7
        # eps = 2

        batch_size = real.shape[0]
        
        real1 = real.clone() #real[0].cpu().detach().numpy()
        fake1 = fake.clone() #fake[0].cpu().detach().numpy()
        ###### debug purposes ########
        # fake1[real1==0] = 1.0
        # a = np.asarray(real1.cpu().detach()*7000)[0]
        # # a[a!=0.0] = 10000
        # b = np.asarray(fake1.cpu().detach()*7000)[0]
        # plt.imshow(np.uint16(b), vmin=0, vmax=7000)
        # plt.colorbar()
        # plt.savefig(save_dir +'fake1_'+str(epoch)+'.png',bbox_inches='tight')
        # plt.close()

        # plt.imshow(np.uint16(a), vmin=0, vmax=7000)
        # plt.colorbar()
        # plt.savefig(save_dir +'real1_'+str(epoch)+'.png',bbox_inches='tight')
        # plt.close()
        # b[b!=0.0] = 10000
        # c = np.abs(a-b)
        # cv2.imwrite(save_dir+'gttest_'+str(epoch)+'.png',np.uint16(a))
        # cv2.imwrite(save_dir+'faketest_'+str(epoch)+'.png',np.uint16(b))
        # cv2.imwrite(save_dir+'diff_test_'+str(epoch)+'.png',np.uint16(c))
        ####################################

        # real1[real1==0] = eps
        # fake1[fake1==0] = eps



        # if epoch > 24:
        #     all_real_pcd = self.point_cloud(fake1[0]).clone() * 1000.0
        # else:    

        all_real_pcd = self.point_cloud(real1[0]).clone() * 1000.0
        all_fake_pcd = self.point_cloud(fake1[0]).clone() * 1000.0

        all_real_pcd[all_real_pcd==0] = eps
        all_fake_pcd[all_fake_pcd==0] = eps

        # real_pcd = nan_real_pcd[~torch.isnan(nan_real_pcd)]
        # fake_pcd = nan_fake_pcd[~torch.isnan(nan_real_pcd)]

        
      
        #######################
        # Take out nan points #
        # If this doesn't work replace the values with 2 or something
        ### loss17
        nan_z_real = all_real_pcd[:,2].clone()
        temp_z_real = nan_z_real[~torch.isnan(nan_z_real)]
       
        nan_z_fake = all_fake_pcd[:,2].clone()
        temp_z_fake = nan_z_fake[~torch.isnan(nan_z_real)]
        
        nan_x_real = all_real_pcd[:,0].clone()
        temp_x_real = nan_x_real[~torch.isnan(nan_x_real)]
        
        nan_x_fake = all_fake_pcd[:,0].clone()
        temp_x_fake = nan_x_fake[~torch.isnan(nan_x_real)]
        
        nan_y_real = all_real_pcd[:,1].clone()
        temp_y_real = nan_y_real[~torch.isnan(nan_y_real)]
        
        nan_y_fake = all_fake_pcd[:,1].clone()
        temp_y_fake = nan_y_fake[~torch.isnan(nan_y_real)]

        z_real = temp_z_real[~torch.isnan(temp_z_fake)]
        z_fake = temp_z_fake[~torch.isnan(temp_z_fake)]

        x_real = temp_x_real[~torch.isnan(temp_x_fake)]
        x_fake = temp_x_fake[~torch.isnan(temp_x_fake)]

        y_real = temp_y_real[~torch.isnan(temp_y_fake)]
        y_fake = temp_y_fake[~torch.isnan(temp_y_fake)]
        

        lossX = torch.sqrt(torch.mean(torch.abs(x_real-x_fake)**2))
        lossZ = torch.sqrt(torch.mean(torch.abs(z_real-z_fake)**2))
        lossY = torch.sqrt(torch.mean(torch.abs(y_real-y_fake)**2))
        
       
        RMSE_log = 10000* torch.sqrt(torch.mean(torch.abs(torch.log(torch.abs(z_real))-torch.log(torch.abs(z_fake)))**2))

        loss17 = RMSE_log * torch.abs(10*(3-torch.exp(1*lossX)-torch.exp(1*lossY)-torch.exp(1*lossZ)))
        
        # plane_mean_dist_grad = torch.tensor(0.0001).cuda() # ????????
        
        # ###############################
        # Curv loss
        # ################################
        nan_cloud = all_fake_pcd.clone()
        nan_cloud[torch.isnan(nan_cloud)] = 0

        norm_all_fake_pcd = nan_cloud/nan_cloud.max()
        # norm_o3d_pcd_fake = o3d.geometry.PointCloud()
        # norm_o3d_pcd_fake.points = o3d.utility.Vector3dVector(norm_all_fake_pcd.cpu().detach().numpy())
        
        
        # o3d.io.write_point_cloud("fake_norm_cloud"+str(epoch)+".pcd", norm_o3d_pcd_fake)

        # ################# with Open3d
        # fake_plane_model, fake_inliers = norm_o3d_pcd_fake.segment_plane(distance_threshold=0.025,
        #                                     ransac_n=3,
        #                                     num_iterations=500)
        # [a, b, c, d] = fake_plane_model
        # a_torch = torch.from_numpy(np.array(a)).cuda()
        # b_torch = torch.from_numpy(np.array(b)).cuda()
        # c_torch = torch.from_numpy(np.array(c)).cuda()
        # d_torch = torch.from_numpy(np.array(d)).cuda()

        # fake_plane_pcd = norm_o3d_pcd_fake.select_by_index(fake_inliers)
        # fake_plane_pcd.paint_uniform_color([1.0, 0, 0])
        # ################################

        # if loss17<400:
        #     np_plane_model, np_plane_inliers = self.oh_numpy_RANSAC_give_me_a_plane(np.asarray(norm_o3d_pcd_fake.points),thresh=0.025,
        #                                                                             minPoints=5000,
        #                                                                             maxIteration=1000)
        thresh = torch.tensor(0.025 - 0.001*epoch).cuda()
        if thresh <= 0.005: thresh = torch.tensor(0.005).cuda()
        torch_plane_model, torch_plane_inliers = self.oh_torch_RANSAC_give_me_a_plane(norm_all_fake_pcd,thresh=thresh,
                                                                                    minPoints=5000,
                                                                                    maxIteration=1000)
        # o3d.io.write_point_cloud("fake_plane_o3d"+str(epoch)+".pcd", fake_plane_pcd)
        # fake_plane_pcd = norm_o3d_pcd_fake.select_by_index(torch_plane_inliers.cpu().detach().numpy())
        # o3d.io.write_point_cloud("plane_torch"+str(epoch)+".pcd", fake_plane_pcd)

        # torch_pcd.points = o3d.utility.Vector3dVector(norm_all_fake_pcd[fake_inliers].cpu().detach().numpy())
        # o3d.io.write_point_cloud("fake_plane_torch_"+str(epoch)+".pcd", torch_pcd)
        
        # Open3D translation and rotation
        ################# removable stuff, just for testing
        # fake_plane_pcd = fake_plane_pcd.translate((0,0,d/c))
        # # o3d.io.write_point_cloud("fake_plane_o3d_trans_"+str(epoch)+".pcd", fake_plane_pcd)

        # cos_theta = c / math.sqrt(a**2 + b**2 + c**2)
        # sin_theta = math.sqrt((a**2+b**2)/(a**2 + b**2 + c**2))
        # u_1 = b / math.sqrt(a**2 + b**2 )
        # u_2 = -a / math.sqrt(a**2 + b**2)
        # pred_rotation_matrix = np.array([[cos_theta + u_1**2 * (1-cos_theta), u_1*u_2*(1-cos_theta), u_2*sin_theta],
        #                         [u_1*u_2*(1-cos_theta), cos_theta + u_2**2*(1- cos_theta), -u_1*sin_theta],
        #                         [-u_2*sin_theta, u_1*sin_theta, cos_theta]])

        # center_before_rot = fake_plane_pcd.get_center()
        # fake_plane_pcd.rotate(pred_rotation_matrix)
        # ###############################
        # center_after_rot = fake_plane_pcd.get_center()

        # o3d.io.write_point_cloud("fake_plane_o3d_rot_"+str(epoch)+".pcd", fake_plane_pcd)
        
        #####
        # fake_plane_pcd = fake_plane_pcd.translate((-center_before_rot[0],-center_before_rot[1],-center_before_rot[2]))
        # center_after_translate = fake_plane_pcd.get_center()
        # fake_plane_pcd.rotate(pred_rotation_matrix,center=(center_before_rot[0],center_before_rot[1],center_before_rot[2]))
        # fake_plane_pcd = fake_plane_pcd.translate((center_before_rot[0],center_before_rot[1],center_before_rot[2]))
        # o3d.io.write_point_cloud("fake_plane_o3d_rot_zero_c_and_t"+str(epoch)+".pcd", fake_plane_pcd)
        ######
        
        ###################
        a_torch = torch_plane_model[0]
        b_torch = torch_plane_model[1]
        c_torch = torch_plane_model[2]
        d_torch = torch_plane_model[3]

        # torch translation and rotation
        torch_fake_plane = norm_all_fake_pcd[torch_plane_inliers]
        torch_translation = torch.tensor([0, 0, d_torch/c_torch]).cuda()
        torch_fake_plane += torch_translation

        # torch_pcd.points = o3d.utility.Vector3dVector(torch_fake_plane.cpu().detach().numpy())
        # o3d.io.write_point_cloud("fake_plane_torch_trans_"+str(epoch)+".pcd", torch_pcd)

        cos_theta_torch = c_torch / torch.sqrt(a_torch**2 + b_torch**2 + c_torch**2)
        sin_theta_torch = torch.sqrt((a_torch**2+b_torch**2)/(a_torch**2 + b_torch**2 + c_torch**2))
        u_1_torch = b_torch / torch.sqrt(a_torch**2 + b_torch**2 )
        u_2_torch = -a_torch / torch.sqrt(a_torch**2 + b_torch**2)
        
        pred_rotation_matrix_torch = torch.tensor([[cos_theta_torch + u_1_torch**2 * (1-cos_theta_torch), u_1_torch*u_2_torch*(1-cos_theta_torch), u_2_torch*sin_theta_torch],
                                [u_1_torch*u_2_torch*(1-cos_theta_torch), cos_theta_torch + u_2_torch**2*(1- cos_theta_torch), -u_1_torch*sin_theta_torch],
                                [-u_2_torch*sin_theta_torch, u_1_torch*sin_theta_torch, cos_theta_torch]]).cuda()
        # torch_plane_center = torch.tensor([center_before_rot[0],center_before_rot[1],center_before_rot[2]]).cuda()
        # torch_fake_plane = torch_fake_plane - torch_plane_center
        pred_rotation_matrix_torch = pred_rotation_matrix_torch.transpose(0,1)
        torch_fake_plane = torch.matmul(torch_fake_plane,pred_rotation_matrix_torch)
        # torch_fake_plane += torch_plane_center

        # if loss17 < 400:
        #     torch_pcd = o3d.geometry.PointCloud()
        #     torch_pcd.points = o3d.utility.Vector3dVector(torch_fake_plane.cpu().detach().numpy())
        #     o3d.io.write_point_cloud("fake_plane_torch_rot_"+str(epoch)+".pcd", torch_pcd)

        #########
        # BELOW #
        #########
        # fake_plane_numpy = np.asarray(fake_plane_pcd.points)
        # fake_plane_numpy[:,2] = fake_plane_numpy[:,2] - fake_plane_numpy[:,2].max() 
        # plane_mean_distance_below_XY_numpy = np.mean(abs(fake_plane_numpy[:,2]))
        # fake_plane_numpy[:,2] = fake_plane_numpy[:,2] - fake_plane_numpy[:,2].min() 
        # plane_mean_distance_above_XY_numpy = np.mean(abs(fake_plane_numpy[:,2]))
        
        torch_fake_plane_dist_below = torch_fake_plane[:,2] - torch_fake_plane[:,2].max()
        plane_mean_distance_below_XY = torch.mean(abs(torch_fake_plane_dist_below))

        ##############
        # ABOVE #
        #########

        torch_fake_plane_dist_above = torch_fake_plane[:,2] - torch_fake_plane[:,2].min()
        plane_mean_distance_above_XY = torch.mean(abs(torch_fake_plane_dist_above))
        
        if plane_mean_distance_above_XY == 0: plane_mean_distance_above_XY = torch.tensor(0.0000001).cuda()
        plane_mean_dist_grad = 1000* (plane_mean_distance_above_XY + plane_mean_distance_below_XY)
        


        loss_curv = loss17 + plane_mean_dist_grad
        delta = [RMSE_log, lossX, lossY, lossZ, plane_mean_dist_grad, loss17]


        return delta, loss_curv

    def oh_numpy_RANSAC_give_me_a_plane(self, pts, thresh=0.05, minPoints=100, maxIteration=1000):
        """
        Find the best equation for a plane.
        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the plane which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :returns:
        - `self.equation`:  Parameters of the plane using Ax+By+Cy+D `np.array (1, 4)`
        - `self.inliers`: points from the dataset considered inliers
        ---
        """
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):

            # Samples 3 random points
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]

            # We have to find the plane equation described by those 3 points
            # We find first 2 vectors that are part of this plane
            # A = pt2 - pt1
            # B = pt3 - pt1

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]

            # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
            vecC = np.cross(vecA, vecB)

            # The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
            # We have to use a point to find k
            vecC = vecC / np.linalg.norm(vecC)
            k = -np.sum(np.multiply(vecC, pt_samples[1, :]))
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

            # Distance from a point to a plane
            # https://mathworld.wolfram.com/Point-PlaneDistance.html
            pt_id_inliers = []  # list of inliers ids
            dist_pt = (
                plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] + plane_eq[2] * pts[:, 2] + plane_eq[3]
            ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
            if len(pt_id_inliers) > len(best_inliers) & (len(pt_id_inliers) > minPoints):
                best_eq = plane_eq
                best_inliers = pt_id_inliers
            self.inliers = best_inliers
            self.equation = best_eq

        return self.equation, self.inliers

    def oh_torch_RANSAC_give_me_a_plane(self, pts, thresh=0.05, minPoints=100, maxIteration=1000):
        """
        Find the best equation for a plane.
        :param pts: 3D point cloud as a `torch.tensor (N,3)`.
        :param thresh: Threshold distance from the plane which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :returns:
        - `self.equation`:  Parameters of the plane using Ax+By+Cy+D `torch.tensor (1, 4)`
        - `self.inliers`: points from the dataset considered inliers
        ---
        """
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):

            # Samples 3 random points
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]

            # We have to find the plane equation described by those 3 points
            # We find first 2 vectors that are part of this plane
            # A = pt2 - pt1
            # B = pt3 - pt1

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]

            # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
            vecC = torch.cross(vecA, vecB)

            # The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
            # We have to use a point to find k
            vecC = vecC / torch.linalg.norm(vecC)
            k = -torch.sum(torch.multiply(vecC, pt_samples[1, :]))
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

            # Distance from a point to a plane
            # https://mathworld.wolfram.com/Point-PlaneDistance.html
            pt_id_inliers = []  # list of inliers ids
            dist_pt = (
                plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] + plane_eq[2] * pts[:, 2] + plane_eq[3]
            ) / torch.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = torch.where(torch.abs(dist_pt) <= thresh)[0]
            if (len(pt_id_inliers) > len(best_inliers)) & (len(pt_id_inliers) > minPoints):
                best_eq = plane_eq
                best_inliers = pt_id_inliers
            self.inliers = best_inliers
            self.equation = best_eq

        return self.equation, self.inliers

    def point_cloud(self, depth1):
        """Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.

        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.

        """
        # depth is of shape (1,480,640)
        cx = 334.081
        cy = 169.808
        fx = 460.585
        fy = 460.268

        if depth1.shape[0] == 3:
            depth = depth1[2].clone()
        else:
            depth = depth1[0].clone()
        # open3d_img = o3d.t.geometry.Image(depth[0])#/1000.0)
        # intrinsics = o3d.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy)
        # pcd = o3d.geometry.create_point_cloud_from_depth_image(open3d_img,intrinsic=intrinsics)
        
        rows, cols = depth.shape
        c, _ = torch.meshgrid(torch.arange(cols), torch.arange(cols))
        c = torch.meshgrid(torch.arange(cols))
        new_c = c[0].reshape([1,cols]).to('cuda')
        r = torch.meshgrid(torch.arange(rows))
        new_r = r[0].unsqueeze(-1).to('cuda')
        valid = (depth > 0) & (depth < 65535)
        nan_number = torch.tensor(np.nan).to('cuda')
        # zero_number = torch.tensor(0.).to('cuda')
        z = torch.where(valid, depth/1000.0, nan_number) # allways divide with 1000.0
        x = torch.where(valid, z * (new_c - cx) / fx, nan_number)
        y = torch.where(valid, z * (new_r - cy) / fy, nan_number)
        

        dimension = rows * cols
        z_ok = z.reshape(dimension)
        x_ok = x.reshape(dimension)
        y_ok = y.reshape(dimension)
    
        return torch.stack((x_ok,y_ok,z_ok),dim=1) 

    def image_from_cloud(self, point_cloud):
        
        cx = 334.081
        cy = 169.808
        fx = 460.585
        fy = 460.268
        
        # point_cloud = point_cloud/1000.0
        np_image = np.tile(0,(360,640))

        z = point_cloud[:,2] * 1000.0
        x = point_cloud[:,0]
        y = point_cloud[:,1]

        valid = ~(np.isnan(z) | (z==0))
        z = np.where(valid, z*1000.0, 0)
        # z[np.isnan(z) | (z==0)] = 1e-7
        # pos_x = (point_cloud[:,0] * 1000.0 * fx)/ z + cx
        valid_x = ~(np.isnan(x) | np.isnan(z) | (z==0))
        pos_x = np.where(valid_x,(x * 1000.0 * fx)/ z + cx, 0)
        pos_x = pos_x.astype(np.int32)
        # pos_y = (point_cloud[:,1] * 1000.0 * fy)/ z + cy
        valid_y = ~(np.isnan(y) | np.isnan(z) | (z==0))
        pos_y = np.where(valid_y,(y * 1000.0 * fy)/z + cy, 0) 
        pos_y = pos_y.astype(np.int32)
        
        pos_x[pos_x>639] = 639
        pos_x[pos_x<0] = 0
        pos_y[pos_y>359] =359
        pos_y[pos_y<0] = 0

        pos_x = pos_x.reshape(360,640)
        pos_y = pos_y.reshape(360,640)
        z = z.reshape(360,640)
        np_image[pos_y,pos_x] = z

        # depth = depth.cpu().detach().numpy()
        # rows, cols = depth[0].shape
        # c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        # valid = (depth[0] > 0) & (depth[0] < 65535)
        # z = np.where(valid, depth[0] / 1000.0, np.nan)
        # x = np.where(valid, z * (c - cx) / fx, 0)
        # y = np.where(valid, z * (r - cy) / fy, 0)
        return np_image
 

def adjust_learning_rate(optimizer, decay=0.1):
    """Sets the learning rate to the initial LR decayed by 0.5 every 20 epochs"""
    for param_group in optimizer.param_groups:
        param_group['lr'] = decay * param_group['lr']

def parse_args():
    """
    Parse input arguments
    """
    parser = argparse.ArgumentParser(
        description='Single image depth estimation')
    parser.add_argument('--dataset', dest='dataset',
                        help='training dataset',
                        default='nyuv2', type=str)
    parser.add_argument('--epochs', dest='max_epochs',
                        help='number of epochs to train',
                        default=10, type=int)
    parser.add_argument('--cuda', dest='cuda',
                        help='whether use CUDA',
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
                        default='saved_models_t7', type=str)

# config optimization
    parser.add_argument('--o', dest='optimizer',
                        help='training optimizer',
                        default="adam", type=str)
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
                        default=False, type=bool)
    parser.add_argument('--start_at', dest='start_epoch',
                        help='epoch to start with',
                        default=0, type=int)
    parser.add_argument('--checksession', dest='checksession',
                        help='checksession to load model',
                        default=1, type=int)
    parser.add_argument('--checkepoch', dest='checkepoch',
                        help='checkepoch to load model',
                        default=1, type=int)
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


if __name__ == '__main__':

    args = parse_args()

    if torch.cuda.is_available() and not args.cuda:
        print("WARNING: You might want to run with --cuda")

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    train_dataset = MyCustomDataset()
    train_size = len(train_dataset)
    eval_dataset = MyCustomDataset(train=False)
    eval_size = len(eval_dataset)
    print(train_size)
    print(eval_size)

    train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=args.bs,
                                                   shuffle=True, num_workers=args.num_workers)

    
    eval_dataloader = torch.utils.data.DataLoader(eval_dataset, batch_size=args.bs,
                                                  shuffle=True, num_workers=args.num_workers) #maybe trying with False for shuffle, here?
   
    
    # network initialization
    print('Initializing model...')
    i2d = I2D(fixed_feature_weights=False)
    torch.cuda.empty_cache()
    if args.cuda:
        i2d = i2d.cuda()

    print('Done!')

    # hyperparams
    lr = args.lr
    bs = args.bs
    lr_decay_step = args.lr_decay_step
    lr_decay_gamma = args.lr_decay_gamma

    # params
    params = []
    for key, value in dict(i2d.named_parameters()).items():
        if value.requires_grad:
            if 'bias' in key:
                DOUBLE_BIAS = 0
                WEIGHT_DECAY = 4e-5
                params += [{'params': [value], 'lr':lr*(DOUBLE_BIAS + 1),
                            'weight_decay': 4e-5 and WEIGHT_DECAY or 0}]
            else:
                params += [{'params': [value], 'lr':lr, 'weight_decay': 4e-5}]

    # optimizer
    if args.optimizer == "adam":
        optimizer = torch.optim.Adam(params, lr=lr, betas=(0.9, 0.999), eps=1e-08, weight_decay=4e-5)
    elif args.optimizer == "sgd":
        optimizer = torch.optim.SGD(params, lr=lr, momentum=0.9)

    dddDepth_criterion = DDDDepthDiff()

    # resume
    if args.resume:
        load_name = os.path.join(args.output_dir,
                                 'i2d_1_{}.pth'.format(args.checkepoch))
        print("loading checkpoint %s" % (load_name))
        state = i2d.state_dict()
        checkpoint = torch.load(load_name)
        args.start_epoch = checkpoint['epoch']
        checkpoint = {k: v for k,
                      v in checkpoint['model'].items() if k in state}
        state.update(checkpoint)
        i2d.load_state_dict(state)
        # optimizer.load_state_dict(checkpoint['optimizer'])
        # lr = optimizer.param_groups[0]['lr']
        if 'pooling_mode' in checkpoint.keys():
            POOLING_MODE = checkpoint['pooling_mode']
        print("loaded checkpoint %s" % (load_name))
        del checkpoint
        torch.cuda.empty_cache()

    # constants
    iters_per_epoch = int(train_size / args.bs)

    grad_factor = 10.
    normal_factor = 1.
    # max_depth = 6571
    max_depth = 11000
    
    #for visualizing the train and validation loss
    train_loss_arr = []
    val_loss_arr = []

    for epoch in range(args.start_epoch, args.max_epochs):

        train_loss = 0 
        val_loss = 0
        # setting to train mode
        i2d.train()
        start = time.time()
        if epoch % (args.lr_decay_step + 1) == 0:
            adjust_learning_rate(optimizer, args.lr_decay_gamma)
            lr *= args.lr_decay_gamma

        img = Variable(torch.FloatTensor(1))
        z = Variable(torch.FloatTensor(1))
        if args.cuda:
            img = img.cuda()
            z = z.cuda()

        train_data_iter = iter(train_dataloader)
        show_image = True
        # saving results in a txt file
        save_dir = '/home/marian/workspace/monodepth_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/training_process_t7/'

        
        for step in range(iters_per_epoch):
            start = time.time()
            data = train_data_iter.next()

            img.resize_(data[0].size()).copy_(data[0])#*max_depth)
            # z.resize_(data[1].size()).copy_(data[1])#*max_depth)
            # one channel depth image
            z.resize_(data[0][:,0].unsqueeze(0).size()).copy_(data[0][:,0].unsqueeze(0))

            # max_depth = data[1].max()

            optimizer.zero_grad()
            z_fake = i2d(img)#*max_depth # * 6000 #z.max()
            
            delta, dddDepth_loss = dddDepth_criterion(z_fake,z,epoch,show_image)#*max_depth,z*max_depth)

            
            torch.autograd.set_detect_anomaly(True)


            loss = 1*dddDepth_loss
            # loss *= 10
            loss.backward()
            optimizer.step()
            writer.add_scalar("Loss/train",loss,step)
            train_loss += loss.item()
            end = time.time()
            
            if show_image:
                o3d_pcd = o3d.geometry.PointCloud()

                ##############################
                #####save input cloud#########
                input_img = img[0][2].cpu().numpy()
                rgbArray = input_img*max_depth # np.array(img[0].cpu()*max_depth,np.uint16).transpose((1,2,0))
                plt.imshow(rgbArray, vmin=0, vmax=max_depth)
                plt.colorbar()
                plt.savefig(save_dir+'depthCV_'+str(epoch)+'.png', bbox_inches='tight')
                plt.close()
                # cv2.imwrite(save_dir+'depthirCV_'+str(epoch)+'.png',rgbArray)
                
                input_depth = img.clone() 
                input_pcd = dddDepth_criterion.point_cloud(input_depth[0]).cpu().detach().numpy()
                o3d_pcd.points = o3d.utility.Vector3dVector(input_pcd*max_depth)
                o3d.io.write_point_cloud(save_dir+"input_cloud"+str(epoch)+".pcd", o3d_pcd)
                # a = cv2.imread(save_dir+'depthirCV_'+str(epoch)+'.png', cv2.IMREAD_UNCHANGED)
                # vmin, vmax = 0, 10000/65536.

                ####################
                #depth ground truth#
                plt.imshow(z[0].cpu().numpy().transpose((1,2,0))*max_depth, vmin=0, vmax=max_depth)
                plt.colorbar()
                plt.savefig(save_dir +'gt_'+str(epoch)+'.png',bbox_inches='tight')
                plt.close()
                # plt.imshow(z[0].cpu().numpy().transpose((1,2,0)))#, vmin=vmin, vmax=vmax)
                # plt.colorbar()
                # plt.savefig(save_dir +'unscaled_gt_'+str(epoch)+'.png',bbox_inches='tight')
                # plt.close()
                z_pcd = dddDepth_criterion.point_cloud(z[0]).cpu().detach().numpy()
                o3d_pcd.points = o3d.utility.Vector3dVector(z_pcd*max_depth)
                o3d.io.write_point_cloud(save_dir+"gt_cloud"+str(epoch)+".pcd", o3d_pcd)
                
                ##################
                #depth prediction#
                plt.imshow(z_fake[0].cpu().detach().numpy().transpose((1,2,0))*max_depth, vmin=0, vmax=max_depth)
                plt.colorbar()
                plt.savefig(save_dir +'pred_'+str(epoch)+'.png',bbox_inches='tight')
                plt.close()
                # plt.imshow(z_fake[0].cpu().detach().numpy().transpose((1,2,0)))#, vmin=vmin, vmax=vmax)
                # plt.colorbar()
                # plt.savefig(save_dir +'unscaled_pred_'+str(epoch)+'.png',bbox_inches='tight')
                # plt.close()
                z_fake_pcd = dddDepth_criterion.point_cloud(z_fake[0]).cpu().detach().numpy()
                o3d_pcd.points = o3d.utility.Vector3dVector(z_fake_pcd*max_depth)
                o3d.io.write_point_cloud(save_dir+"pred_cloud"+str(epoch)+".pcd", o3d_pcd)
                
                #### save difference ####
                plt.imshow(np.abs(z[0].cpu().numpy().transpose((1,2,0)) - z_fake[0].cpu().detach().numpy().transpose((1,2,0)))*max_depth)
                plt.colorbar()
                plt.savefig(save_dir+'diff_'+str(epoch)+'.png', bbox_inches='tight')
                plt.close()

                pred_img = z_fake[0][0].cpu().detach().numpy()

                plt.imshow(np.abs(input_img - pred_img)*max_depth)
                plt.colorbar()
                plt.savefig(save_dir+'input_pred_diff_'+str(epoch)+'.png', bbox_inches='tight')
                plt.close()

                # z_diff = dddDepth_criterion.point_cloud(torch.abs(z[0]-z_fake[0])).cpu().detach().numpy()
                # o3d_pcd.points = o3d.utility.Vector3dVector(z_diff*max_depth)
                # o3d.io.write_point_cloud(save_dir+"diff_cloud"+str(epoch)+".pcd", o3d_pcd)
                

                show_image=False


            # info
            if step % args.disp_interval == 0:
                # file_object = open("/home/marian/workspace/monodepth_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/results.txt", 'a')
                print("[epoch %2d][iter %4d] loss: %.4f 3DDepthLoss: %.4f RMSE: %.4f lossX: %.4f lossY: %.4f lossZ: %.4f curv_loss: %.4f loss17: %.4f"#RMSElog: %.4f Grad: %.4f Normals diff: %.4f"
                      % (epoch, step, loss, dddDepth_loss, delta[0], delta[1], delta[2], delta[3], delta[4], delta[5]))


        if epoch%4 == 0:
            save_name = os.path.join(args.output_dir, 'i2d_{}_{}.pth'.format(args.session, epoch))
            
            torch.save({'epoch': epoch+1,
                        'model': i2d.state_dict(),
                        #                     'optimizer': optimizer.state_dict(),
                        },
                    save_name)
            

        print('save model: {}'.format(save_name))
        print('time elapsed: %fs' % (end - start))

        # if epoch % 1 == 0:
        with torch.no_grad():
            # setting to eval mode
            i2d.eval()

            # img = Variable(torch.FloatTensor(1), volatile=True)
            # img = Variable(torch.FloatTensor(1),requires_grad=False)
            img = Variable(torch.FloatTensor(1))
            # z = Variable(torch.FloatTensor(1), volatile=True)
            # z = Variable(torch.FloatTensor(1), requires_grad=False)
            z = Variable(torch.FloatTensor(1))
            if args.cuda:
                img = img.cuda()
                z = z.cuda()

            print('evaluating...')

            rmse_accum = 0
            count = 0
            eval_data_iter = iter(eval_dataloader)
            for i, data_eval in enumerate(eval_data_iter):
                print(i, '/', len(eval_data_iter)-1)

                img.resize_(data_eval[0].size()).copy_(data_eval[0])
                # z.resize_(data_eval[1].size()).copy_(data_eval[1])
                z.resize_(data_eval[0][:,0].unsqueeze(0).size()).copy_(data_eval[0][:,0].unsqueeze(0))

                z_fake = i2d(img)

                
                
                delta_val, dddDepth_loss_eval = dddDepth_criterion(z_fake,z,epoch,show_image)
                loss_val = 1*dddDepth_loss_eval
                writer.add_scalar("Loss/validation",loss_val,i)

                val_loss += loss_val.item()
                # print("Loss on test_data: ",loss_eval)
                if i==337:
                    plt.imshow(z[0].cpu().numpy().transpose((1,2,0))*max_depth)#, vmin=vmin, vmax=vmax)
                    plt.colorbar()
                    plt.savefig('/home/marian/workspace/monodepth_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/main_multiP/val_vis_images/gt_'+str(epoch)+'.png',bbox_inches='tight')
                    plt.close()

                    plt.imshow(z_fake[0].cpu().detach().numpy().transpose((1,2,0))*max_depth)#, vmin=vmin, vmax=vmax)
                    plt.colorbar()
                    plt.savefig('/home/marian/workspace/monodepth_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/main_multiP/val_vis_images/pred_'+str(epoch)+'.png',bbox_inches='tight')
                    plt.close()


            train_loss = train_loss/iters_per_epoch #len(train_dataloader)
            val_loss = val_loss/len(eval_dataloader)

            train_loss_arr.append(train_loss)
            val_loss_arr.append(val_loss)
            print('Epoch: {} \tTraining Loss: {:.6f} \tValidation Loss: {:.6f}'.format(epoch, train_loss, val_loss))

            file_object = open("/home/marian/workspace/monodepth_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/results_t7.txt", 'a')
            # print("[epoch %2d][iter %4d] loss: %.4f RMSElog: %.4f"# grad_loss: %.4f"# normal_loss: %.4f"
            #         % (epoch, step, loss, depth_loss))#, grad_loss))#, normal_loss))
            # print("[epoch %2d][iter %4d] loss: %.4f RMSElog: %.4f"
            #       % (epoch, step, loss, depth_loss))
            file_object.write('\nEpoch: {} \tTraining Loss: {:.6f} \tValidation Loss: {:.6f}'.format(epoch, train_loss, val_loss)) #grad_loss: %.4f" # normal_loss: %.4f" 
                    # % (epoch, step, loss, depth_loss))#, grad_loss))#, normal_loss))
            file_object.close()
            writer.flush()
            # print("[epoch %2d] RMSE_log: %.4f RMSE: %.4f"
            #       % (epoch, torch.sqrt(eval_loss/count), torch.sqrt(rmse_accum/count)))
            # with open('val.txt', 'a') as f:
            #     f.write("[epoch %2d] RMSE_log: %.4f RMSE: %.4f\n"
            #             % (epoch, torch.sqrt(eval_loss/count), torch.sqrt(rmse_accum/count)))
        

    writer.close()
    epochs = range(args.start_epoch, args.max_epochs)
    plt.plot(epochs, train_loss_arr, '-g', label='Training loss')
    plt.plot(epochs, val_loss_arr, 'b', label='Validation loss')
    plt.title('Training and Validation loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.savefig(save_dir+"t100_losses.png")
    plt.close()