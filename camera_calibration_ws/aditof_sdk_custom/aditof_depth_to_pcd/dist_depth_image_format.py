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
    camera_header = camera_info_msg.header


def callback(image_msg):
    global info_pub
    global image_pub
    global camera_header

    try:
        pass
    except Exception as err:
        print(err)

    frame_depths = np.fromstring(image_msg.data, dtype='uint16')
    min_range_f = min(frame_depths)
    max_range_f = max(frame_depths)
    delta_f = max_range_f - min_range_f
    print("Frame---------------------------")
    print("min_range: ", min_range_f, "max_range: ", max_range_f, "delta: ", delta_f)
    
    frame = bridge.imgmsg_to_cv2(image_msg,'mono16') 
    # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    min_range = np.min(frame) #0
    max_range = np.max(frame) #65535
    delta = max_range - min_range
    print("cv_bridge---------------------------")
    print('min_range: ',min_range,'max_range: ', max_range, "delta: ", delta)
    
    # print(frame)
    img2 = np.array(frame, dtype=np.float32)
    # print("center depth: ", img2[320,240])
    # print(img2[0,0])
    # img2_vec = np.concatenate(img2)
    # min_range = min(img2_vec[0], img2_vec[len(img2)-1])
    # min_range = np.min(img2)
    # print(img2)
    min_range = 0 #np.min(img2) #0
    max_range = 4500 #np.max(img2) #65535 
    delta = max_range - min_range
    print("after conversion (np.float32)----------")
    print('min_range: ', min_range,'max_range: ', max_range,'delta: ', delta)
    print("######################################")
    img2 = img2/1000 #(img2 - min_range)/delta * 4.5
    # img2_not_ready_yet = (img2 - min_range)/delta 
    # img2 = img2_not_ready_yet * 65535 + (1.0 - img2_not_ready_yet) * 0
    # print(img2) 
    # print(img2)
    # for it in range(479):
    #     for it2 in range(639):
    #         norm_val = (img2[it,it2] - min_range) / delta
    #         img2[it,it2] = norm_val
     
    cv2.waitKey(1)
    
    
    img2_msg = bridge.cv2_to_imgmsg(img2,'32FC1')
    img2_msg.header = camera_header
    # img2_msg.data.decode()
    # data = np.linspace(img2_msg.data. , img2_msg.data + img2_msg.width * img2_msg.height)
    # min_val = min(data.index(0), data.index(len(data)))
    # max_val = max(data.index(0), data.index(len(data)))
    # delta = max_val - min_val

    # for it in img2_msg.width*img2_msg.height:
    #     norm_val = (data[it] - min_val) / delta
    #     img2_msg.data[it] = norm_val
    

    image_pub.publish(img2_msg)
    #image_time_ref_msg.source = "image_msg"
    #image_time_ref_msg.header.stamp = 
    
    #image_pub.publish(bridge.cv2_to_imgmsg(img2,'16UC4'))
    #image_pub.publish(image)
    #info_pub.publish(camera_info_msg)


def start_node():
    global info_pub
    global image_pub
    
    rospy.init_node('dist_depth_format')
    rospy.loginfo('dist to depth format started')
    
    image_sub = rospy.Subscriber("/aditof_roscpp/aditof_depth", Image, callback)
    info_sub = rospy.Subscriber("/aditof_roscpp/aditof_camera_info", CameraInfo, callbackCamera)
    
    #ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1, allow_headerless=False)
    #ts.registerCallback(callback_sync)

    image_pub = rospy.Publisher('/aditof_dist_depth_format', Image, queue_size=1)
    #info_pub = rospy.Publisher('/aditof_camera_info_sync', CameraInfo, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass