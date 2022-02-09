import matplotlib.pyplot as plt
from model_fpn import I2D
from threading import Thread
from torch.autograd import Variable
from torchvision.utils import save_image
import argparse, time
import cv2
import numpy as np
import os, sys
import timeit
import torch, time
from torchvision import transforms
import open3d as o3d

model_version = "single_plane_i2d_1_24.pth"
model_version_folder = "final_tests_single_plane"
raw_PNG_images = "c24_robo.PNG"
pcd_folder = "c24_robo.PCD"
pred_pcd_folder = "c24_robo.PCD.pred"
in_pred_image_pairs = "c24_robo_pairs"

def parse_args():
    """
    Parse input arguments
    """
    parser = argparse.ArgumentParser(description='Normal image estimation from ToF depth image')
    parser.add_argument('--cuda', dest='cuda',
                      help='whether use CUDA',
                      default=True,
                      action='store_true')
    parser.add_argument('--num_workers', dest='num_workers',
                      help='num_workers',
                      default=1, type=int)  
    parser.add_argument('--input_image_path', dest='input_image_path',
                      help='path to a single input image for evaluation',
                    #   default='/media/rambo/ssd2/marian/datasets/monodepth/training_data/depth_synth/', type=str)
                      default='/home/marian/workspace/monodepth_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/curvature_grad/'+ model_version_folder +'/'+raw_PNG_images +'/', type=str)
    parser.add_argument('--eval_folder', dest='eval_folder',
                      help='evaluate only one image or the whole folder',
                      default=True, type=bool)
    parser.add_argument('--model_path', dest='model_path',
                      help='path to the model to use',
                      default='/home/marian/workspace/monodepth_ws/monodepth-FPN/good_models/'+ model_version, type=str)
                    #   default='/home/marian/calibration_ws/monodepth-FPN/saved_models/i2d_1_24.pth', type=str)
                    #   default='/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/saved_models/t90_i2d_1_24.pth', type=str)

    args = parser.parse_args()
    return args

def point_cloud(depth1):
    """Transform a depth image into a point cloud with one point for each
    pixel in the image, using the camera transform for a camera
    centred at cx, cy with field of view fx, fy.

    depth is a 2-D ndarray with shape (rows, cols) containing
    depths from 1 to 254 inclusive. The result is a 3-D array with
    shape (rows, cols, 3). Pixels with invalid depth in the input have
    NaN for the z-coordinate in the result.

    """
    # depth is of shape (1,480,640)
    # K = [460.58518931365654, 0.0, 334.0805877590529, 0.0, 460.2679961517268, 169.80766383231037, 0.0, 0.0, 1.0] # pico zense
    K = [460.585, 0.0, 334.081, 0.0, 460.268, 169.808, 0.0, 0.0, 1.0] # pico zense
    # K = [582.62448167737955, 0.0, 313.04475870804731, 0.0, 582.69103270988637, 238.44389626620386, 0.0, 0.0, 1.0] # nyu_v2_dataset
    # K = [582.624, 0.0, 313.045, 0.0, 582.691, 238.444, 0.0, 0.0, 1.0] # nyu_v2_dataset
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]

    depth = depth1.clone()
    # open3d_img = o3d.t.geometry.Image(depth[0])#/1000.0)
    # intrinsics = o3d.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy)
    # pcd = o3d.geometry.create_point_cloud_from_depth_image(open3d_img,intrinsic=intrinsics)
    
    rows, cols = depth[0].shape
    c, _ = torch.meshgrid(torch.arange(cols), torch.arange(cols))
    c = torch.meshgrid(torch.arange(cols))
    new_c = c[0].reshape([1,cols]).to('cuda')
    r = torch.meshgrid(torch.arange(rows))
    new_r = r[0].unsqueeze(-1).to('cuda')
    valid = (depth[0] > 0) & (depth[0] < 65535)
    nan_number = torch.tensor(np.nan).to('cuda')
    zero_number = torch.tensor(0.).to('cuda')
    z = torch.where(valid, depth[0]/1000.0, nan_number) # allways divide with 1000.0
    x = torch.where(valid, z * (new_c - cx) / fx, nan_number)
    y = torch.where(valid, z * (new_r - cy) / fy, nan_number)
    

    dimension = rows * cols
    z_ok = z.reshape(dimension)
    x_ok = x.reshape(dimension)
    y_ok = y.reshape(dimension)

    return torch.stack((x_ok,y_ok,z_ok),dim=1) 

if __name__ == '__main__':

    args = parse_args()

    if torch.cuda.is_available() and not args.cuda:
        print("WARNING: You might want to run with --cuda")
    
    # network initialization
    print('Initializing model...')
    i2d = I2D(fixed_feature_weights=False)
    if args.cuda:
        i2d = i2d.cuda()
        
    print('Done!')
    
    
    load_name = os.path.join(args.model_path)
    print("loading checkpoint %s" % (load_name))
    state = i2d.state_dict()
    checkpoint = torch.load(load_name)
    checkpoint = {k: v for k, v in checkpoint['model'].items() if k in state}
    state.update(checkpoint)
    i2d.load_state_dict(state)
    if 'pooling_mode' in checkpoint.keys():
        POOLING_MODE = checkpoint['pooling_mode']
    print("loaded checkpoint %s" % (load_name))
    del checkpoint
    torch.cuda.empty_cache()

    i2d.eval()

    img = Variable(torch.FloatTensor(1))

    print('evaluating...')
    if args.eval_folder:
        dlist=os.listdir(args.input_image_path)
        dlist.sort()
        time_sum = 0
        counter = 0
        # max_depth=10000.
        min_depth=300.
        nan_number = torch.tensor(np.nan).to('cuda')
        eps_number = torch.tensor(1e-7).to('cuda')
        zero_number = torch.tensor(0.).to('cuda')
        for filename in dlist:
            if filename.endswith(".png"):
                path=args.input_image_path+filename
                print("Predicting for:"+filename)
                depth = cv2.imread(path,-1)
                depth = cv2.resize(depth,(640,360))
                orig_depth = depth
                depth=depth.astype(np.float32)
                
                if len(depth.shape) < 3:
                    print("Got 1 channel depth images, creating 3 channel depth images")
                    combine_depth = np.empty((depth.shape[0],depth.shape[1], 3))
                    combine_depth[:,:,0] = depth
                    combine_depth[:,:,1] = depth
                    combine_depth[:,:,2] = depth
                    depth = combine_depth

                depth2 = np.moveaxis(depth,-1,0)
                depth = np.array(depth,np.float32,copy=True).transpose((2,0,1)) 
                # img = torch.from_numpy(depth2).float().unsqueeze(0).cuda()
                max_depth = depth.max()
                img = torch.from_numpy(depth).long().unsqueeze(0).cuda()/max_depth
                # img=img.unsqeeze(0)
                
                start = timeit.default_timer()
                # img2=img.clone()
                # img[img>max_depth] = max_depth
                # img[img<min_depth] = zero_number
                # imgmask=img.clone()
                # imgmask=imgmask[:,0,:,:].unsqueeze(1)
                # valid = (imgmask > 0) & (imgmask < max_depth+1)
                # img2=img2-min_depth
                # m_depth=torch.max(img)
                # img=img/max_depth                 
                z_fake = i2d(img)
                # z_fake = torch.where(valid, z_fake*max_depth, zero_number)
                stop = timeit.default_timer()
                time_sum=time_sum+stop-start
                counter=counter+1
                save_path= path.replace(raw_PNG_images, pcd_folder)[:-4]

                o3d_pcd = o3d.geometry.PointCloud()
                z_orig_pcd = point_cloud(torch.unsqueeze(img[0][0],dim=0)).cpu().detach().numpy()
                o3d_pcd.points = o3d.utility.Vector3dVector(z_orig_pcd)
                o3d.io.write_point_cloud(save_path +'_orig.pcd', o3d_pcd)
                
                save_path_images = path.replace(raw_PNG_images, in_pred_image_pairs)[:-4]
                #  just for a faire (normalized) compare
                plt.imshow(orig_depth/max_depth, vmin=0)
                plt.colorbar()
                plt.savefig(save_path_images +'_orig.png',bbox_inches='tight')
                plt.close()
                # ######################

                save_path= path.replace(raw_PNG_images, pred_pcd_folder)[:-4]
                o3d_pcd = o3d.geometry.PointCloud()
                z_fake_pcd = point_cloud(z_fake[0]).cpu().detach().numpy()
                o3d_pcd.points = o3d.utility.Vector3dVector(z_fake_pcd)
                o3d.io.write_point_cloud(save_path +'_pred.pcd', o3d_pcd)

                plt.imshow(z_fake[0].cpu().detach().numpy().transpose((1,2,0)), vmin=0)
                plt.colorbar()
                plt.savefig(save_path_images +'_pred.png',bbox_inches='tight')
                plt.close()
                # npimage=(z_fake[0]/max_depth).squeeze(0).cpu().detach().numpy().astype(np.uint16)
                # cv2.imwrite(save_path +'_pred.png', npimage)

            else:
                continue
        print('Predicting '+str(counter)+' images took ', time_sum/counter)  
    else:
        depth = cv2.imread(args.input_image_path,cv2.IMREAD_UNCHANGED).astype(np.float32)
        if len(depth.shape) < 3:
            print("Got 1 channel depth images, creating 3 channel depth images")
            combine_depth = np.empty((depth.shape[0],depth.shape[1], 3))
            combine_depth[:,:,0] = depth
            combine_depth[:,:,1] = depth
            combine_depth[:,:,2] = depth
            depth = combine_depth
        depth2 = np.moveaxis(depth,-1,0)
        img = torch.from_numpy(depth2).float().unsqueeze(0)
        start = timeit.default_timer()
        z_fake = i2d(img.cuda())
        stop = timeit.default_timer()
        zfv=z_fake*2-1
        z_fake_norm=zfv.pow(2).sum(dim=1).pow(0.5).unsqueeze(1)
        zfv=zfv/z_fake_norm
        z_fake=(zfv+1)/2
        save_path=args.input_image_path[:-4]
        save_image(z_fake[0], save_path +"_pred"+'.png')
        print('Predicting the image took ', stop-start)
    

