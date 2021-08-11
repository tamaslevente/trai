# to run: python3 server_ros.py or... run it with debugger if you want to stop it when you want
# otherwise you should give it a Ctrl+C and then send to it another point cloud from blender

import rospy
import socket
import pickle
import numpy as np

from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2
import ros_numpy


HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)


def pointcloud_to_pointcloud_ros_msg(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    # cloud_arr = np.atleast_2d(cloud_arr)

    cloud_for_ros_numpy = np.zeros(cloud_arr.shape[0],dtype=[('x',np.float32),('y',np.float32),('z',np.float32)])

    cloud_for_ros_numpy['x'] = cloud_arr[:,0]
    cloud_for_ros_numpy['y'] = cloud_arr[:,1]
    cloud_for_ros_numpy['z'] = cloud_arr[:,2]

    cloud_msg = ros_numpy.msgify(PointCloud2, cloud_for_ros_numpy)

    header = Header()

    if stamp is None:
        header.stamp = rospy.Time().now()
    else:
        header.stamp = stamp

    if frame_id is None:
        header.frame_id = "map"
    else:
        header.frame_id = frame_id


    cloud_msg.header = header
   
    return cloud_msg 

def process_point_cloud():
    rospy.init_node('map', anonymous=True)
    rate = rospy.Rate(3) # 10hz
    pub = rospy.Publisher('/best_position', String, queue_size=1)
    cloud_pub = rospy.Publisher('/blender_pointcloud', PointCloud2, queue_size=1)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        i = 0
        print('Connected by', addr)
        data = []
        valid = False
        packet = []

        while not rospy.is_shutdown():
            packet = conn.recv(2**32)
            if not packet: # and valid: # after all the (point cloud pieces) packets are received process the point cloud and publish something
                print(data.__len__())
                point_cloud = pickle.loads(b"".join(data))
                print("Point cloud ready to go, with shape: ",point_cloud.shape)
                position = "THIS!... might be a good position... unfortunately I don't have such knowledge"
                pub.publish(position)
                cloud_pub.publish(pointcloud_to_pointcloud_ros_msg(point_cloud))
                print("Point cloud published!")
                data = []
                valid = False
                # elif not packet : # if no packets are received wait until there are PACKETS!!!
                # cloud_pub.publish(pointcloud_to_pointcloud_ros_msg(point_cloud))
                s.listen()
                conn, addr = s.accept()
            else: # keep appending the packets into a nice big pointcloud
                print("Appending...")
                data.append(packet)
                valid = True

            rate.sleep()

if __name__ == '__main__':
    try:
        process_point_cloud()
    except rospy.ROSInterruptException:
        pass