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

# def array_to_point_cloud(points, parent_frame):
#     """ Creates a point cloud message.
#     Args:
#         points: Nx3 array of xyz positions (m)
#         parent_frame: frame in which the point cloud is defined
#     Returns:
#         sensor_msgs/PointCloud2 message
#     """
#     ros_dtype = sensor_msgs.msg.PointField.FLOAT32
#     dtype = np.float32
#     itemsize = np.dtype(dtype).itemsize

#     data = points.astype(dtype).tobytes()
    
#     fields = [sensor_msgs.msg.PointField(
#         name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
#         for i, n in enumerate('xyz')]

#     header = std_msgs.msg.Header(frame_id=parent_frame, stamp=rospy.Time.now())

#     return sensor_msgs.msg.PointCloud2(
#         header=header,
#         height=360,#1,
#         width= 640,#points.shape[0],
#         is_dense=False,
#         is_bigendian=False,
#         fields=fields,
#         point_step= 16, #(itemsize * 3),
#         row_step= 10240, #(itemsize * 3 * points.shape[0]),
#         data=data
#     )

# def array_to_point_cloudRGBA(points, parent_frame):
#     """ Creates a point cloud message.
#     Args:
#         points: Nx7 array of xyz positions (m) and rgba colors (0..1)
#         parent_frame: frame in which the point cloud is defined
#     Returns:
#         sensor_msgs/PointCloud2 message
#     """
#     ros_dtype = sensor_msgs.msg.PointField.FLOAT32
#     dtype = np.float32
#     itemsize = np.dtype(dtype).itemsize

#     data = points.astype(dtype).tobytes()
    
#     fields = [sensor_msgs.msg.PointField(
#         name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
#         for i, n in enumerate('xyzrgba')]

#     header = std_msgs.msg.Header(frame_id=parent_frame, stamp=rospy.Time.now())

#     return sensor_msgs.msg.PointCloud2(
#         header=header,
#         height=1,
#         width=points.shape[0],
#         is_dense=False,
#         is_bigendian=False,
#         fields=fields,
#         point_step=(itemsize * 7),
#         row_step=(itemsize * 7 * points.shape[0]),
#         data=data
#     )


# def point_cloud(depth1):
#         """Transform a depth image into a point cloud with one point for each
#         pixel in the image, using the camera transform for a camera
#         centred at cx, cy with field of view fx, fy.

#         depth is a 2-D ndarray with shape (rows, cols) containing
#         depths from 1 to 254 inclusive. The result is a 3-D array with
#         shape (rows, cols, 3). Pixels with invalid depth in the input have
#         NaN for the z-coordinate in the result.

#         """
#         # depth is of shape (1,480,640)
#         cx = 334.081
#         cy = 169.808
#         fx = 460.585
#         fy = 460.268

#         depth = depth1.clone()
#         # open3d_img = o3d.t.geometry.Image(depth[0])#/1000.0)
#         # intrinsics = o3d.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy)
#         # pcd = o3d.geometry.create_point_cloud_from_depth_image(open3d_img,intrinsic=intrinsics)
        

#         depth = depth.cpu().detach().numpy()
#         rows, cols = depth[0].shape
#         c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
#         valid = (depth[0] > 0) & (depth[0] < 65535)
#         z = np.where(valid, depth[0] / 1000.0, np.nan)
#         x = np.where(valid, z * (c - cx) / fx, 0)
#         y = np.where(valid, z * (r - cy) / fy, 0)

#         dimension = rows * cols
#         x_ok = x.reshape(dimension)
#         y_ok = y.reshape(dimension)
#         z_ok = z.reshape(dimension)
#         return np.dstack((x_ok,y_ok,z_ok))

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