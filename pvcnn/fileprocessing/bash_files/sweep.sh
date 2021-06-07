#!bin/bash
builddir=$PWD'/../build/'
input_dir=/media/rambo/ssd2/Szilard/nyu_v2_filter/dataset_plane/pcd_vox/
output_dir=/media/rambo/ssd2/Szilard/nyu_v2_filter/dataset_plane/pcd_vox/


if [$1 -eq '']
then 
    cd $input_dir
else
    input_dir=$1
    cd $input_dir
fi

for filename in *.pcd; do
    cd $builddir
    ./sweep_nan $input_dir $output_dir $filename
done