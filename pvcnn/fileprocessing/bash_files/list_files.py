import os
import numpy as np
import argparse
import cv2

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--dir',dest='dir', default="/media/rambo/ssd2/Szilard/nyu_v2_tofnest/nyu_v2/pcdgt/",
                    help='the directory to the source files')
parser.add_argument('--test',dest='test', default=False,
                    help='do you need to create separate test/train/validation datasets?')
args = parser.parse_args()

directory=args.dir
dlist=os.listdir(directory)
dlist.sort()

f = open(directory+"filelist.txt", "w")
ftest = open(directory+"test.txt", "w")
ftrain = open(directory+"train.txt", "w")
fvalid = open(directory+"validset.txt", "w")
ending=".xyz"
n=0
for filename in dlist:
    if filename.endswith(ending):
        f.write(filename[:-len(ending)]+"\n")
        if args.test:
            
            if n%10:
                ftrain.write('data/obj/'+filename+"\n")
            # if n==4:
            #     ftest.write(filename[:-len(ending)]+"\n")
            else:
                # fvalid.write(filename[:-len(ending)]+"\n")
                ftest.write('data/obj/'+filename+"\n")
            n=n+1
    else:
        continue
f.close()
ftrain.close()
ftest.close()
fvalid.close()