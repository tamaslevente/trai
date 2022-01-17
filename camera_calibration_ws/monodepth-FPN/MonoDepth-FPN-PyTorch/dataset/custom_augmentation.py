"""
A script for "multiplying" the data set. ;)
"""
import os
import cv2
import imgaug as ia

rgb_directory = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/main_multiP/input_irdd_data/"
                 
def horizontal_flip(img,img_name):
    flipped_img = ia.augmenters.fliplr(img)
    cv2.imwrite(img_name+"hor_flip.png",flipped_img)

def hor_and_ver_flip(img,img_name):
    flipped_img = ia.augmenters.fliplr(img)
    flipped_img = ia.augmenters.flipud(flipped_img)
    cv2.imwrite(img_name+"hor_and_ver_flip.png",flipped_img)

def vertical_flip(img,img_name):
    flipped_img = ia.augmenters.flipud(img)
    cv2.imwrite(img_name+"ver_flip.png",flipped_img)

# def rotate_cw(img, img_name, rot_degree):
#     rotate = ia.augmenters.Affine(rotate=rot_degree)
#     image_rot = rotate(image=img)    
#     cv2.imwrite(img_name+"_rot_cw.png",image_rot)
#     # translate(image_rot,imgname+"_rotcw.png")

# def rotate_ccw(img, img_name,rot_degree):
#     rotate = ia.augmenters.Affine(rotate=rot_degree)
#     image_rot = rotate(image=img)    
#     cv2.imwrite(img_name+"_rot_ccw.png",image_rot)
#     # imageio.imwrite(directory+imgname+"_rotcw.png",image_rot, format='PNG-FI')
#     # translate(image_rot,imgname+"_rotcw.png")

dlist = os.listdir(rgb_directory)
dlist.sort()
# print(dlist)
for filename in dlist:
    if filename.endswith(".png"):
        print("Image:"+filename)
        rgb_path = rgb_directory+filename
        rgb = cv2.imread(rgb_path,-1)
        # depth_path = rgb_path.replace('input_ddd_data', 'depth_gt_filled_data')
        # depth = cv2.imread(depth_path,-1)
        
        rgb_image_name = rgb_path[:-4]
        # depth_image_name =  depth_path[:-4]

        horizontal_flip(rgb,rgb_image_name)
        # horizontal_flip(depth,depth_image_name)
        
        vertical_flip(rgb,rgb_image_name)
        # vertical_flip(depth,depth_image_name)

        hor_and_ver_flip(rgb, rgb_image_name)
        # hor_and_ver_flip(depth, depth_image_name)

        ############################
        # this is not ok yet!!!!!! and... will probably never be. :))
        # rot_degree = random.randint(85,95)
        # rotate_cw(rgb,rgb_image_name,rot_degree)
        # rotate_cw(depth,depth_image_name,rot_degree)
        
        # anti_rot_degree = random.randint(-95,-85)
        # rotate_ccw(rgb,rgb_image_name, anti_rot_degree)
        # rotate_ccw(depth,depth_image_name, anti_rot_degree)

        print()

    else:
        continue 
