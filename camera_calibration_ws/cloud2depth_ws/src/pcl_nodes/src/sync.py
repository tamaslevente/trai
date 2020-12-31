import rospy
from sensor_msgs.msg import Image, CameraInfo, TimeReference
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

bridge = CvBridge()
#image_time_ref_msg = TimeReference()
camera_header = None 

def callbackCamera(camera_info_msg):
    global camera_header
    global info_pub
    camera_header = camera_info_msg.header
    info_pub.publish(camera_info_msg)


def callback(image_msg):
    global info_pub
    global image_pub
    global camera_header

    try:
        pass
    except Exception as err:
        print(err)

    frame = bridge.imgmsg_to_cv2(image_msg,'mono16') 
    
    img2 = np.array(frame, dtype=np.float32)

    img2 = img2/1000
     
    cv2.waitKey(1)
    
    img2_msg = bridge.cv2_to_imgmsg(img2,'32FC1')
    img2_msg.header = camera_header

    image_pub.publish(img2_msg)


def start_node():
    global info_pub
    global image_pub
    
    rospy.init_node('depth_format')
    rospy.loginfo('depth format started')
    
    image_sub = rospy.Subscriber("/aditof_roscpp/aditof_depth", Image, callback)
    info_sub = rospy.Subscriber("/aditof_roscpp/aditof_camera_info", CameraInfo, callbackCamera)

    image_pub = rospy.Publisher('/aditof_depth_sync', Image, queue_size=1)
    info_pub = rospy.Publisher('/aditof_camera_info_sync', CameraInfo, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass