import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os
import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--dir', default="",
                    help='the directory to the source files')
args = parser.parse_args()

directory=args.dir
images=[]

dlist=os.listdir(directory)
dlist.sort()
for filename in dlist:
    if filename.endswith(".jpg") or filename.endswith(".png"):
        #print(os.path.join(directory, filename))
        images.append(filename)
    else:
        continue

n=0
for i in range(len(images)):
    image=cv2.imread(directory+images[i])
    filename=f'{n:04d}'
    #print(filename)
    if n%3 == 0 :
        cv2.imwrite(directory+"test/"+filename+".png", image) 
    else:
        cv2.imwrite(directory+"train/"+filename+".png", image)
    n=n+1