#!bin/bash
bashdir=$PWD
builddir=$PWD'/../build/'
# cd $1

# mkdir pcd_pred/
# mkdir mask/
# mkdir pcd_pred_mask/

# cd $bashdir
# python3 rename.py --dir=$1depth_pred/ --ext=.png
# # bash depth2pcd.sh $1depth_pred/ $1pcd_pred/
# python3 rename.py --dir=$1pcd_pred/ --ext=.pcd
# bash create_mask.sh $1depth_pred/ $1mask/
# python3 rename.py --dir=$1mask/ --ext=.png
# bash depthmask2pcd.sh $1depth/ $1mask/ $1pcd_pred_mask/
# python3 rename.py --dir=$1pcd_pred_mask/ --ext=.pcd

echo Prediction
bash pw_compare.sh own pcd