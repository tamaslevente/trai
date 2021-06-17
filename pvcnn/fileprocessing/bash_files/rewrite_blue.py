import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

#directoryd = input("Please enter the path to the depth folder:\n")
#directoryi = input("Please enter the path to the ir folder:\n")
#directorys = input("Please enter the path to the folder:\n")
directory = "/home/szilard/rosbag_files/normalrgb_data/depthir_b0/"
directorymod = "/home/szilard/rosbag_files/normalrgb_data/depthir_b1/"
images=[]

dlist=os.listdir(directory)
dlist.sort()
for filename in dlist:
    if filename.endswith(".jpg") or filename.endswith(".png"):
        #print(os.path.join(directory, filename))
        images.append(filename)
    else:
        continue

print(len(images))
for i in range(len(images)):
    img=cv2.imread(directory+images[i])
    rgbArray = np.zeros((len(img),len(img[0]),3), 'uint8')  
    for z in range(len(img)):
        for j in range(len(img[1])):
            rgbArray[z][j][0] = img[z][j][2]
            rgbArray[z][j][1] = img[z][j][1]
            rgbArray[z][j][2] = 1
    
    im = Image.fromarray(np.uint8(rgbArray))
    print(images[i])
    #image.show()
    im.save(directorymod+images[i])