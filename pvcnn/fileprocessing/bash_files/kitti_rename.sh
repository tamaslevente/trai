#!bin/bash
builddir=$PWD'/../build/'
input_dir=/media/rambo/ssd2/Szilard/kitti/train/
inter2=/proj_depth/velodyne_raw/image_02/
inter3=/proj_depth/velodyne_raw/image_03/
output_dir=/media/rambo/ssd2/Szilard/kitti/tofnest/depth/


if [[ ! -z "$1" ]] 
then 
    input_dir=$1
    if [[ ! -z "$2" ]] 
    then 
        output_dir=$2
    fi
fi
cd $input_dir
n=0
for dirname in *; do
    # echo $input_dir$dirname
    cd $input_dir$dirname$inter2
    for filename in *.png; do
        # echo $input_dir$dirname$inter2$filename
        foo=$(printf "%05d" $n)
        n=$((n+1))
        echo $foo
        cp $input_dir$dirname$inter2$filename $output_dir$foo.png
    done
    cd $input_dir$dirname$inter3
    for filename in *.png; do
        # echo $input_dir$dirname$inter2$filename
        foo=$(printf "%05d" $n)
        n=$((n+1))
        echo $foo
        cp $input_dir$dirname$inter3$filename $output_dir$foo.png
    done
done