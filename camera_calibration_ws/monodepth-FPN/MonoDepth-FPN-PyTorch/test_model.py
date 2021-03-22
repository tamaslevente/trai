import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, TimeReference, PointCloud
import std_msgs.msg
import sensor_msgs
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge 
import cv2
import numpy as np
import ros_numpy

from torch.autograd import Variable
import torch

from model_fpn import I2D
from main_fpn import DDDDepthDiff as ddd_diff


bridge = CvBridge()


def predict_images(image):
    global max_depth
    max_depth = image.max()

    rgb_depth = np.zeros((3,len(image),len(image[0])), 'uint16')
    rgb_depth[0] = image
    rgb_depth[1] = image
    rgb_depth[2] = image
    depth_rgb = np.array(rgb_depth,np.float32,copy=True)

    depth_tensor = torch.from_numpy(depth_rgb).long().unsqueeze(0)/max_depth
    input = Variable(depth_tensor)
    input = input.to(device)
    prediction = i2d(input)
    return prediction
        

def callback(image_msg):
    depth_image = bridge.imgmsg_to_cv2(image_msg)

    # image
    prediction = predict_images(depth_image)
    image = prediction[0].cpu().detach().numpy().transpose((1,2,0))*max_depth
    predicted_image = bridge.cv2_to_imgmsg(image)
    
    predicted_image.header = image_msg.header
    # point cloud
    # predicted_cloud = point_cloud(prediction[0])
   
    # pred_pcd_msg = array_to_point_cloud(predicted_cloud[0],parent_frame="pico_zense_depth_frame")
    # publish all
    # cloud_pub.publish(pred_pcd_msg)
    image_pub.publish(predicted_image)

def start_node():
    global image_pub
    global cloud_pub
    global i2d
    global device
    # global pred_pcd_msg

    rospy.init_node('test_model')
    rospy.loginfo('We start the model testing!')
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    print("Initializing model...")
    i2d = I2D(fixed_feature_weights=False)
    i2d = i2d.cuda()
    
    model_name = '/home/marian/calibration_ws/monodepth-FPN/saved_models/i2d_1_24.pth' 
    state = i2d.state_dict()
    checkpoint = torch.load(model_name)
    state.update(checkpoint['model'])
    i2d.load_state_dict(state)
    
    del checkpoint
    torch.cuda.empty_cache()

    i2d.eval()
    print("Done!")

    # pred_pcd_msg = PointCloud()

    image_sub = rospy.Subscriber("/pico_zense/depth/image_raw", Image, callback)

    image_pub = rospy.Publisher('/predicted_depth_images', Image, queue_size=1)
    # cloud_pub = rospy.Publisher("/predicted_cloud",PointCloud2,queue_size=1)

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass