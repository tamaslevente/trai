#!bin/bash
# kitti dataset dir: /media/rambo/ssd2/Szilard/kitti/tofnest/
bashdir=$PWD
builddir=$PWD'/../build/'
basedir=/media/rambo/ssd2/Szilard/kitti/tofnest_selection/

if [[ ! -z "$1" ]] 
then 
    basedir=$1
    
fi

cd $basedir


# rm -rf depth3/
# mkdir depth3
# rm -rf pcd/
# mkdir pcd
# rm -rf pcd_normals/
# mkdir pcd_normals
# rm -rf pcd_ncolor/
# mkdir pcd_ncolor
# rm -rf rgb_normals/
# mkdir rgb_normals
rm -rf pcd_pred/
mkdir pcd_pred

cd $bashdir
# bash depth3.sh $basedir'depth/' $basedir'depth3/'
# # bash depth2pcd.sh $basedir'depth/' $basedir'pcd/'
# # bash compute_normals.sh $basedir'pcd/' $basedir'pcd_normals/'
# # bash normal2rgb.sh $basedir'pcd_normals/' $basedir'pcd_ncolor/'
# # bash cloud2rgbimage.sh $basedir'pcd_ncolor/' $basedir'rgb_normals/'
# # python3 rename.py --dir=$basedir'depth/' --ext=.png
# # python3 rename.py --dir=$basedir'pcd_normals/' --ext=.pcd

# cd $basedir
# rm -rf dataset/
# mkdir dataset
# mkdir dataset/depth
# mkdir dataset/rgb
# mkdir dataset/depth/test
# mkdir dataset/depth/train
# mkdir dataset/rgb/test
# mkdir dataset/rgb/train

# cp depth3/* dataset/depth/
# cp rgb_normals/* dataset/rgb/


# cd $bashdir
# python3 create_train.py --dir=$basedir'dataset/depth/'
# python3 create_train.py --dir=$basedir'dataset/rgb/'

#eval
python3 rename.py --dir=$basedir'predictions/' --ext=.png
bash depth2pcd_normal.sh $basedir'depth/' $basedir'predictions/' $basedir'pcd_pred/'
bash kitti_normal_performance.sh