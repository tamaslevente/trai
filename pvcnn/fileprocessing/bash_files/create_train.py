import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os
import argparse
import shutil

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
        
# file1 = open(directory+"filelist.txt" ,'r')
# Lines = file1.readlines()
n=0

# for line in Lines:
#     # image=cv2.imread(directory+images[i],cv2.IMREAD_UNCHANGED )
#     image_name=line.strip()
#     filename=f'{n:05d}'
#     print(line.strip())
#     if n%3 == 0 :
#         # cv2.imwrite(directory+"test/"+filename+".png", image.astype(np.uint16)) 
#         shutil.copy2(directory+image_name,directory+"test/"+filename+".png")
#     else:
#         # cv2.imwrite(directory+"train/"+filename+".png", image.astype(np.uint16))
#         shutil.copy2(directory+image_name,directory+"train/"+filename+".png")
#     n=n+1

for i in range(len(images)):    
    # image=cv2.imread(directory+images[i],cv2.IMREAD_UNCHANGED )
    image_name=images[i]
    filename=f'{n:05d}'
    print(filename)
    if n%5 == 0 :
        # cv2.imwrite(directory+"test/"+filename+".png", image.astype(np.uint16)) 
        shutil.move(directory+image_name,directory+"test/"+filename+".png")
    else:
        # cv2.imwrite(directory+"train/"+filename+".png", image.astype(np.uint16))
        shutil.move(directory+image_name,directory+"train/"+filename+".png")
    n=n+1