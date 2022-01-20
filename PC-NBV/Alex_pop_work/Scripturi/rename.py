import os

path = "/media/cuda/60D6E001D6DFD57E/Alex/valid"

for root, dirs, files in os.walk(path):
    for i in files:
        if  ".pcd" in str(i):
            os.rename(os.path.join(root, i), os.path.join(root, "1" + ".pcd"))