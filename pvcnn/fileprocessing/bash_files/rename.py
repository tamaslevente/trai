
import os
import argparse
import shutil


parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--dir', default="/media/rambo/ssd2/Szilard/kitti/tofnest/depth/",
                    help='the directory to the source files')
parser.add_argument('--ext', default=".png",
                    help='the extension of file')
parser.add_argument('--ending', default="",
                    help='the ending name of a file')
args = parser.parse_args()

directory=args.dir

dlist=os.listdir(directory)
dlist.sort()
extension = args.ext

n=0
for filename in dlist:
    if filename.endswith(extension):
        number=f'{n:05d}'
        shutil.move(directory+filename,directory+number+args.ending+extension)
        n=n+1
    else:
        continue

# f = open(directory+"filelist.txt", "r")
# for x in f:
#   shutil.copy2(directory+"rgb/"+x[:-1]+"_rgb.png",directory+x[:-1]+"_rgb.png")