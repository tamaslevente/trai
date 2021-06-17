import gzip
import pickle as cPickle
import json

dir = '/home/szilard/Downloads/nyuv2_surfacenormal_metadata/surfacenormal_metadata/'
fp=gzip.open(dir+'all_normals.pklz','rb')
file_dict=cPickle.load(fp)
normals=file_dict['all_normals']
print("file is opened")
size = normals.shape
for i in range(normals.shape[0]):
    number=f'{i:06d}'
    print(number)
    f = open(dir+number+".normals", "w")
    for j in range(normals.shape[1]):
        for k in range(normals.shape[2]):
            for l in range(normals.shape[3]):
                value = normals[i][j][k][l]
                f.write(str(value)+" ")
            f.write("\n")
    f.close()