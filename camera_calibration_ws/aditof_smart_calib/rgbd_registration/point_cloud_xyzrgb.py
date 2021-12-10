#!/usr/bin/env python

from numpy.core.fromnumeric import transpose
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys
import message_filters
import std_msgs.msg
from numpy import loadtxt

ir_pub = None
rgb_pub = None 

bridge = CvBridge()
    

def callback_sync(): #rgb_msg, pcd_msg, depth_cam_info_msg):

    pcd = loadtxt("/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/rgb_pcd/p0025_cam_pcd.txt", delimiter=" ", unpack=False)

    rgb_img = cv2.imread("/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/cam_P0025/test/res-rgb_p0025_0.png")

    # rgb_not = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding="passthrough") #BAYER_BGGR16    
    # # rgb = np.frombuffer(rgb_msg.data,np.uint16).reshape(rgb_msg.height,rgb_msg.width,1)
    # rgb = cv2.cvtColor(rgb_not, cv2.COLOR_BayerBG2RGB)


    Krgb = np.array([[314.100533382509,             0,            321.814357865278],
                     [0,                    419.882158000649,     236.903650998016],
                     [0,                            0,                   1        ]])

    R = np.array([[ 0.999926329273159,   0.000776708049317,  -0.012113329472651],
                  [-0.000902746171809,   0.999945480696576,  -0.010402917083351],
                  [ 0.012104589032931,   0.010413085954697,   0.999872515156429]])
    
    t = np.array([-36.560085894521158,  -0.888879973399496,  -3.214890777135737])
    t = np.transpose(t)



    n_points= pcd.shape[0]

    scan3d = pcd
    xyz = scan3d[:,:3]
    # transform in mm
    scan3d = pcd*1000
    # xyz = scan3d(:,1:3);
    
    P = np.matmul(np.matmul(Krgb,R),np.concatenate((np.identity(3),np.array([t]).T),axis=1))
    abc = np.matmul(P, np.concatenate((xyz, np.ones((n_points,1))),axis=1).T)
    ab = np.zeros((3, n_points))
    ab = np.round(abc/abc[2,:])
    ab[0,:] = np.where(ab[0,:] <= 0, ab, 100)
    ab[0,:] = np.where(ab[1,:] <= 0, ab[0,:], 100)
    ab[1,:] = np.where(ab[0,:] <= 0, ab[1,:], 100)
    ab[1,:] = np.where(ab[1,:] <= 0, ab[1,:], 100)
    
    print()

    # for i = 1:length(ab)
    # ab(:,i) = round(abc(:,i) / abc(3,i));
    # if ( (ab(1,i) <= 0) || (ab(2,i) <= 0) )
    #     ab(1,i) = 100;
    #     ab(2,i) = 100;
    # end
    # end
    # % scan=project_points2(scan',zeros(3,1),zeros(3,1),...
    # %     handles.camcalibparams.fc,handles.camcalibparams.cc,...
    # %     handles.camcalibparams.kc,handles.camcalibparams.alpha_c);
    # % Flip rows<->columns to get matlab image coordinates, and round off values
    # scan=fliplr(round(ab(1:2,:)'));

    # % Read image
    # % load('aditof_rect_640x480.mat'.mat');
    # % image=imread(image_filename_colorizer);
    # % [rect_image,newOrigin] = undistortImage(image,aditof_rect_640x480.CameraParameters2);
    # % imshow(rect_image);
    # rect_image=imread(image_filename_colorizer);
    # % rect_image = cat(3, rect_not_rgb_image+300, rect_not_rgb_image-500, rect_not_rgb_image);

    # % Initialize empty matrix representing default point color=black
    # scanRGB=zeros(n_points,3);
    # imrows=size(rect_image,1);
    # imcols=size(rect_image,2);

    # % Find indices of all points that project within image
    # inliers=find(scan(:,1)>0 & scan(:,1)<imrows & scan(:,2)>0 ...
    #     & scan(:,2)<imcols);

    # %% For all points that project within the image, lookup the color
    # %% and store in scanRGB
    # % Convert [scan(inliers,1) scan(inliers,2)] to linear index based on size
    # % of image
    # inliers_lindex=sub2ind([imrows imcols],scan(inliers,1),scan(inliers,2));
    # % Convert image from imrows*imcols*3 to (imrows*imcols)*3
    # rect_image=reshape(rect_image,imrows*imcols,3);
    # scanRGB(inliers,:)=rect_image(inliers_lindex,:);
    # scanRGB = cast(scanRGB,'uint32');
    # % scanRGB( find(scan3d(:,1) > 5),: ) = 0;
    # clear scan image inliers inliers_lindex;

    # %% Write VRML file as output
    # fprintf(1,'Writing VRML file rgbScan.wrl\n');
    # fprintf(1,'This may take a minute, so please wait... ');
    # scanRGB_ok = cast(bitor(bitor(bitshift(scanRGB(:,1),16), bitshift(scanRGB(:,2),8)), scanRGB(:,3)),'uint32');
    # scanRGB_float = typecast(scanRGB_ok(:,1),'single');
    # % scanRGB_float = zeros(length(scanRGB_ok),1);
    # % for i=1:length(scanRGB_ok)
    # %     float_element = typecast([scanRGB_ok(i,1) 0],'double');
    # %     float_element = swapbytes(float_element);
    # %     scanRGB_float(i,1) = float_element;
    # % end
    # rgb_pcd_pico = [cast(scan3d,'single'), scanRGB_float];
    # % writematrix(rgb_pcd_pico,'rgbpcd_aditof_rect_640x480_Kc_corrected_3.pcd','Delimiter',' ','FileType','text');
    # writematrix(rgb_pcd_pico,'p0003_all_p0025_params.pcd','Delimiter',' ','FileType','text');


    # alpha = 0.01 # Simple contrast control
    # beta = 1    # Simple brightness control
    # rgb_cont = cv2.convertScaleAbs(rgb, alpha=alpha, beta=beta)
    # rgb_cont = cv2.resize(rgb_cont, (640,480))
    # rgb_cont_msg = bridge.cv2_to_imgmsg(rgb_cont,encoding="bgr8")
    # rgb_cont_msg.header = rgb_msg.header

    

    




def start_node():

    callback_sync()
    # rospy.init_node('point_cloud_xyzrgb_genesis')
    # rospy.loginfo('Conceiving point_cloud_xyzrgb')
    
    # depth_camera_info = message_filters.Subscriber("/aditof_roscpp/aditof_camera_info", CameraInfo)
    # rgb_sub = message_filters.Subscriber("/aditof_roscpp/aditof_rgb", Image)
    # pcd_sub = message_filters.Subscriber("/aditof_roscpp/aditof_pcloud", PointCloud)

    # ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, pcd_sub, depth_camera_info], 1, 0.1, allow_headerless=False)
    # ts.registerCallback(callback_sync)
    

if __name__ == '__main__':
    try:
        start_node()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass