import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, TimeReference
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys
import math
from matplotlib.pylab import plt
import pyvista as pv
from pyvista import examples

import message_filters

net = None
args = None
info_pub = None
image_pub = None

frame_msg = None

bridge = CvBridge()
image_c2d_msg = None 
image_c2d_received = False
image_received = False

def proc():
    global image_pub, image_c2d_msg, frame_msg
    global image_c2d_received, image_received

    image_received = False
    image_c2d_received = False
    frame = bridge.imgmsg_to_cv2(frame_msg,"mono16")
    frame_c2d = bridge.imgmsg_to_cv2(image_c2d_msg,"16UC1")
    # frame_np = np.array(frame, dtype=np.float32)
    diff = frame - frame_c2d
    # (r, c) = np.shape(diff)
    # (X, Y) = np.meshgrid(range(0, c), range(0, r))

    # plt.imshow(frame)
    # plt.show()

    # np.savetxt('depth1.txt',frame)
    # cv2.imwrite('depth_from_cloud.png',frame_c2d)
    cv2.imshow("Frame",frame_c2d)
    
    plt.imshow(frame_c2d)
    plt.show()
    
    plt.imshow(diff)
    plt.show()

    # plt.plot(X,Y, diff[Y,X])

    # frame_np = np.take(frame,range(480))
    # # mesh = pv.examples.load_airplane()

    # print(pv.compare_images(frame,frame_c2d))
    # # mesh = pv.lines_from_points(frame_np,close=False)
    # p = pv.Plotter()
    # p.add_mesh(mesh, color=True)
    # p.show()

    # diff_img = frame_c2d - frame
    # plt.figure()
    # plt.imshow(frame_c2d)
    # plt.matshow(diff_img)
    # plt.colorbar()
    # plt.show()

    # img2 = np.array(frame, dtype=np.float32)

    # img2 = img2/1000
     
    cv2.waitKey(1)
    
    # img2_msg = bridge.cv2_to_imgmsg(img2,'32FC1')
    # img2_msg.header = image_pcloud_header

    # image_pub.publish(img2_msg)


def callbackC2D(image_c2dmsg):
    global image_received, image_c2d_received, image_c2d_msg
    image_c2d_received = True
    image_c2d_msg = image_c2dmsg

    if image_c2d_received and image_received:
        proc()
    # info_pub.publish(camera_info_msg)


def callback(image_msg):
    # global info_pub
    global image_c2d_received, image_received, frame_msg 
    image_received = True
    
    frame_msg = image_msg  

    try:
        pass
    except Exception as err:
        print(err)

    if image_received and image_c2d_received:
        proc()


def start_node():
    # global info_pub
    global image_pub
    
    rospy.init_node('depth_diff')
    rospy.loginfo('depth diff started')
    
    image_sub = rospy.Subscriber("/aditof_roscpp/aditof_depth", Image, callback)
    # info_sub = rospy.Subscriber("/aditof_roscpp/aditof_camera_info", CameraInfo, callbackCamera)
    image_c2d_sub = rospy.Subscriber("/aditof_roscpp/aditof_cloud_2_depth", Image, callbackC2D)

    # image_pub = rospy.Publisher('/aditof_depth_32FC1_format', Image, queue_size=1)
    # info_pub = rospy.Publisher('/aditof_camera_info_32FC1_format', CameraInfo, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass