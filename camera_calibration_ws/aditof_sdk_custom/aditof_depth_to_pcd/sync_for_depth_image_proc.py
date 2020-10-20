import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys

import message_filters

import std_msgs.msg


net = None
args = None
info_pub = None
image_pub = None

#bridge = CvBridge()

def callback_sync(image, camera_info):
    global info_pub
    global image_pub
    

    try:
        pass
    except Exception as err:
        print(err)

    #frame = bridge.imgmsg_to_cv2(image,'rgba8') 
    
    #img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    #img2 = np.array(img, dtype=np.float32)
    #cv2.waitKey(1)

    #image_pub.publish(bridge.cv2_to_imgmsg(img2,'32FC1'))
    image_pub.publish(image)
    info_pub.publish(camera_info)


def start_node():
    global info_pub
    global image_pub

    rospy.init_node('sync_timestamps')
    rospy.loginfo('sync_timestamps started')
    
    info_sub = message_filters.Subscriber("/aditof_roscpp/aditof_camera_info", CameraInfo)
    #image_sub = message_filters.Subscriber("/aditof_roscpp/image_rect_color", Image)

    image_sub = message_filters.Subscriber("/aditof_roscpp/image_rect", Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1, allow_headerless=False)
    ts.registerCallback(callback_sync)

    image_pub = rospy.Publisher('/aditof_image_rect_sync', Image, queue_size=1)
    info_pub = rospy.Publisher('/aditof_camera_info_sync', CameraInfo, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
