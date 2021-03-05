import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

# Don't forget the last "/" !!!!!!!!!!!!!!!!!!
directoryd = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/depth_data/"
directoryi = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/ir_data/"
directorys = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/combined_ddd/"
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

# print(depthimages)
# print(irimages)
n=0
for it in range(len(depthimages)):
    depth=cv2.imread(directoryd+depthimages[it],cv2.CV_16UC1)
    # ir=cv2.imread(directoryi+irimages[it], cv2.CV_16UC1)
    rgbArray = np.zeros((len(depth),len(depth[0]),3), 'uint16')  

    for i in range(len(depth)):
        for j in range(len(depth[i])):
            # rgbArray[i][j][0] = ir[i][j]
            rgbArray[i][j][0] = depth[i][j]
            rgbArray[i][j][1] = depth[i][j]
            rgbArray[i][j][2] = depth[i][j]

    
    cv2.imwrite(directorys+str(n).zfill(5)+".png",rgbArray)
    # image = Image.fromarray(np.uint16(rgbArray))
    #print(image)
    #image.show()
    # image.save(directorys+str(n).zfill(5)+".png")
    print("Image:",str(n).zfill(5)," saved.")
    n=n+1