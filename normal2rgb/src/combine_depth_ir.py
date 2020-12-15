import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

#directoryd = input("Please enter the path to the depth folder:\n")
#directoryi = input("Please enter the path to the ir folder:\n")
#directorys = input("Please enter the path to the folder:\n")
directoryd = "/home/szilard/projects/normalrgb/depthir/depth/"
directoryi = "/home/szilard/projects/normalrgb/depthir/ir/"
directorys = "/home/szilard/projects/normalrgb/depthir/depthir/"
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
for i in range(len(depthimages)):
    depth=cv2.imread(directoryd+depthimages[i], cv2.IMREAD_GRAYSCALE)
    ir=cv2.imread(directoryi+irimages[i], cv2.IMREAD_GRAYSCALE)
    rgbArray = np.zeros((len(depth),len(depth[0]),3), 'uint8')  

    for i in range(len(depth)):
        for j in range(len(depth[i])):
            rgbArray[i][j][0] = ir[i][j]
            rgbArray[i][j][1] = depth[i][j]
    
    image = Image.fromarray(np.uint8(rgbArray))
    #print(image)
    #image.show()
    image.save(directorys+"depthir_"+str(n)+".png")
    n=n+1