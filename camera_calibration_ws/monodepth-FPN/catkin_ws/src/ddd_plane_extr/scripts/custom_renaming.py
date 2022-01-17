import cv2
import os

# Don't forget the last "/" !!!!!!!!!!!!!!!!!!
directory = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/multiP6/combined_ir_d_d_data/ok_images/"
# directory = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/multiP1/depth_gt_filled/ok_images/"
directorys = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/main_multiP/input_irdd_data/"
# directorys = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/main_multiP/depth_gt_filled_data/"
images=[]


dlist=os.listdir(directory)
dlist.sort()
for filename in dlist:
    if filename.endswith(".jpg") or filename.endswith(".png"):
        #print(os.path.join(directory, filename))
        images.append(filename)
    else:
        continue

#first number represents the nexus between the new name and the old set from which it came
n=10000
n=20427
n=30458
n=40512
n=50524
n=60574
for it in range(len(images)):
    img=cv2.imread(directory+images[it],-1)
    cv2.imwrite(directorys+str(n)+".png",img)
    print("Image:",str(n)," saved.")
    n=n+1