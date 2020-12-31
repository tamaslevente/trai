import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, TimeReference
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys
import math

import message_filters

net = None
args = None
info_pub = None
image_pub = None

frame_msg = None

bridge = CvBridge()
image_pcloud_header = None 
image_pcloud_received = False
image_received = False

def proc():
    global image_pub, image_pcloud_header, frame_msg
    global image_pcloud_received, image_received

    image_received = False
    image_pcloud_received = False
    frame = bridge.imgmsg_to_cv2(frame_msg,'mono16')
    img2 = np.array(frame, dtype=np.float32)

    img2 = img2/1000
     
    cv2.waitKey(1)
    
    img2_msg = bridge.cv2_to_imgmsg(img2,'32FC1')
    img2_msg.header = image_pcloud_header

    image_pub.publish(img2_msg)


def callbackPcloud(image_pcloud_msg):
    global image_pcloud_header, image_received, image_pcloud_received
    image_pcloud_received = True
    image_pcloud_header = image_pcloud_msg.header
    
    if image_pcloud_received and image_received:
        proc()
    # info_pub.publish(camera_info_msg)


def callback(image_msg):
    # global info_pub
    global image_pcloud_received, image_received, frame_msg 
    image_received = True
    
    frame_msg = image_msg  

    try:
        pass
    except Exception as err:
        print(err)

    if image_received and image_pcloud_received:
        proc()


def start_node():
    # global info_pub
    global image_pub
    
    rospy.init_node('depth_format')
    rospy.loginfo('depth format started')
    
    image_sub = rospy.Subscriber("/aditof_roscpp/aditof_depth", Image, callback)
    # info_sub = rospy.Subscriber("/aditof_roscpp/aditof_camera_info", CameraInfo, callbackCamera)
    image_pcloud_sub = rospy.Subscriber("/aditof_roscpp/aditof_pcloud", PointCloud2, callbackPcloud)

    image_pub = rospy.Publisher('/aditof_depth_32FC1_format', Image, queue_size=1)
    # info_pub = rospy.Publisher('/aditof_camera_info_32FC1_format', CameraInfo, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass