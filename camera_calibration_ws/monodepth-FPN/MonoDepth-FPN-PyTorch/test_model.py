import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, TimeReference
from cv_bridge import CvBridge 
import cv2
import numpy as np

from torch.autograd import Variable
import torch

from model_fpn import I2D

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

    prediction = predict_images(depth_image)
    predicted_image = bridge.cv2_to_imgmsg(prediction[0].cpu().detach().numpy().transpose((1,2,0))*max_depth)
    
    image_pub.publish(predicted_image)

def start_node():
    global image_pub
    global i2d
    global device

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
 
    image_sub = rospy.Subscriber("/pico_zense/depth/image_raw", Image, callback)

    image_pub = rospy.Publisher('/predicted_depth_images', Image, queue_size=1)
    # info_pub = rospy.Publisher('/aditof_camera_info_32FC1_format', CameraInfo, queue_size=1)


if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass