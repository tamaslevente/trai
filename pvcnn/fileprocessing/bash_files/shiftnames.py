import imageio
imageio.plugins.freeimage.download()
import os
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--dir', default="/media/rambo/ssd2/Szilard/file_repository/1bag_augmented/dataset/depth2ir/",
                    help='the directory to the source files')
args = parser.parse_args()

directory=args.dir
images=[]
imgs=[]
dlist=os.listdir(directory)
dlist.sort()
for filename in dlist:
    if filename.endswith(".jpg") or filename.endswith(".png") or filename.endswith(".pcd") or filename.endswith(".normals"):
        #print(os.path.join(directory, filename))
        images.append(filename)
    else:
        continue

print(len(images))
# n=len(images)
# for i in reversed(range(len(images))):
#     image_name=images[i]
#     print("Image: "+image_name)
#     number=f'{n:05d}'
#     image_name2=number+image_name[5:]
#     print("New name: "+image_name2)
#     n=n-1
#     os.rename(directory+image_name,directory+image_name2)

n=0
for i in range(len(images)):
    image_name=images[i]
    print("Image: "+image_name)
    number=f'{n:05d}'
    image_name2=number+image_name[5:]
    print("New name: "+image_name2)
    n=n+1
    os.rename(directory+image_name,directory+image_name2)
