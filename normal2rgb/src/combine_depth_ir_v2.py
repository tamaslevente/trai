import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as pilimage
import os
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys 

n=0
depth_image=None
ir_image=None

class image_combiner:

    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("pico_zense/depth/image_raw", Image, self.depthcallback)
        self.ir_sub = rospy.Subscriber("pico_zense/ir/image_raw", Image, self.ircallback)

    def depthcallback(self,data):
        depth_image = self.bridge.imgmsg_to_cv2(data, '32FC1')
        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        depth_image=frame*255
    
    def ircallback(self,data):
        ir_image = self.bridge.imgmsg_to_cv2(data, 'mono8')
            
        if depth_image != None:
            rgbArray = np.zeros((len(depth_image),len(depth_image[0]),3), 'uint8')  

            for i in range(len(depth)):
                for j in range(len(depth[i])):
                    rgbArray[i][j][0] = ir[i][j]
                    rgbArray[i][j][1] = depth[i][j]
    
            image = pilimage.fromarray(np.uint8(rgbArray))
            #print(image)
            #image.show()
            image.save("/home/szilard/projects/normalrgb/depthir/depthir2/depthir_"+str(n)+".jpg")
            n=n+1

def main(args):
  ic = image_combiner()
  rospy.init_node('combine', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)