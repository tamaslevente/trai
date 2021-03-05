"""
A script for "multiplying" the data set. ;)
"""
import os
import numpy as np
import cv2
import imgaug as ia

rgb_directory = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/combined_ddd/"
                 
def horizontal_flip(img,img_name):
    flipped_img = ia.augmenters.fliplr(img)
    cv2.imwrite(img_name+"hor_flip.png",flipped_img)

def vertical_flip(img,img_name):
    flipped_img = ia.augmenters.flipud(img)
    cv2.imwrite(img_name+"ver_flip.png",flipped_img)

def rotate_cw(img, img_name):
    rotate = ia.augmenters.Affine(rotate=(85,95))
    image_rot = rotate(image=img)    
    cv2.imwrite(img_name+"_rot_cw.png",image_rot)
    # translate(image_rot,imgname+"_rotcw.png")

def rotate_ccw(img, img_name):
    rotate = ia.augmenters.Affine(rotate=(-95,-85))
    image_rot = rotate(image=img)    
    cv2.imwrite(img_name+"_rot_ccw.png",image_rot)
    # imageio.imwrite(directory+imgname+"_rotcw.png",image_rot, format='PNG-FI')
    # translate(image_rot,imgname+"_rotcw.png")

dlist = os.listdir(rgb_directory)
dlist.sort()
for filename in dlist:
    if filename.endswith(".png"):
        print("Image:"+filename)
        rgb_path = rgb_directory+filename
        rgb = cv2.imread(rgb_path,-1)
        depth_path = rgb_path.replace('combined_ddd', 'depth_gt')
        depth = cv2.imread(depth_path,-1)
        
        rgb_image_name = rgb_path[:-4]
        depth_image_name =  depth_path[:-4]

        horizontal_flip(rgb,rgb_image_name)
        horizontal_flip(depth,depth_image_name)
        
        vertical_flip(rgb,rgb_image_name)
        vertical_flip(depth,depth_image_name)

        rotate_cw(rgb,rgb_image_name)
        rotate_cw(depth,depth_image_name)

        rotate_ccw(rgb,rgb_image_name)
        rotate_ccw(depth,depth_image_name)

        print()
        # img=imageio.imread(directory+image_name, format='PNG-FI')
        # image_name=image_name[:-4]
        # rotaterand(img,image_name)
        # rotatecw(img,image_name)
        # rotateccw(img,image_name)
        # # scale(img,image_name)
        # fliplr(img,image_name)
        # flipud(img,image_name)
    else:
        continue 
