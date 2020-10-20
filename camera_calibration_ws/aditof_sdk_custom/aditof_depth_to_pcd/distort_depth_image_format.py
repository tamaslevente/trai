import rospy
from sensor_msgs.msg import Image, CameraInfo, TimeReference
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys

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
    camera_header = camera_info_msg.header


def callback(image_msg):
    global info_pub
    global image_pub
    global camera_header

    try:
        pass
    except Exception as err:
        print(err)

    frame = bridge.imgmsg_to_cv2(image_msg,'mono16') 
    
    #img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    img2 = np.array(frame, dtype=np.uint16)
    cv2.waitKey(1)
    
    img2_msg = bridge.cv2_to_imgmsg(img2,'16UC1')
    img2_msg.header = camera_header
    image_pub.publish(img2_msg)
    #image_time_ref_msg.source = "image_msg"
    #image_time_ref_msg.header.stamp = 
    
    #image_pub.publish(bridge.cv2_to_imgmsg(img2,'16UC4'))
    #image_pub.publish(image)
    #info_pub.publish(camera_info_msg)


def start_node():
    global info_pub
    global image_pub
    
    rospy.init_node('distort_depth_format')
    rospy.loginfo('distort_depth format started')
    
    image_sub = rospy.Subscriber("/distort_aditof_image_depth_sync", Image, callback)
    info_sub = rospy.Subscriber("/distort_aditof_camera_info_sync", CameraInfo, callbackCamera)

    #ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1, allow_headerless=False)
    #ts.registerCallback(callback_sync)

    image_pub = rospy.Publisher('/distort_aditof_depth_image_format', Image, queue_size=1)
    #info_pub = rospy.Publisher('/aditof_camera_info_sync', CameraInfo, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
