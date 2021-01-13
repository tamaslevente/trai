import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

directoryd = "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/training_data/depth_data_debug/"
directoryi = "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/training_data/ir_data_debug/"
directorys = "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/training_data/combined_ir_depth/"
depthimages=[]
irimages=[]

dlist=os.listdir(directoryd)
dlist.sort()
for filename in dlist:
    if filename.endswith(".jpg") or filename.endswith(".png"):
        #print(os.path.join(directory, filename))
        depthimages.append(filename)
    else:
        continue

ilist=os.listdir(directoryi)
ilist.sort()
for filename in ilist:
    if filename.endswith(".jpg") or filename.endswith(".png"):
        #print(os.path.join(directory, filename))
        irimages.append(filename)
    else:
        continue

print(depthimages)
print(irimages)
n=0
for it in range(len(depthimages)):
    depth=cv2.imread(directoryd+depthimages[it], cv2.IMREAD_GRAYSCALE)
    ir=cv2.imread(directoryi+irimages[it], cv2.IMREAD_GRAYSCALE)
    rgbArray = np.zeros((len(depth),len(depth[0]),3), 'uint8')  

    for i in range(len(depth)):
        for j in range(len(depth[i])):
            rgbArray[i][j][0] = ir[i][j]
            rgbArray[i][j][1] = depth[i][j]
            rgbArray[i][j][2] = depth[i][j]

    
    image = Image.fromarray(np.uint8(rgbArray))
    #print(image)
    #image.show()
    image.save(directorys+"depthir_"+str(n)+".png")
    n=n+1