import numpy as np 
import cv2
import matplotlib.pyplot as plt 

path = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/depth_data/0001_depth.png"
path_repro = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/depth_gt_debug/00001.png"

orig = cv2.imread(path,-1)

repro = cv2.imread(path_repro,-1)
c = np.abs(np.asarray(orig,np.int32)-np.asarray(repro,np.int32))
plt.imsave('diff_2'+'.png',c)
plt.close()
