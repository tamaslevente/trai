#!bin/bash
builddir=$PWD'/../build/'
default_dir=/media/rambo/ssd2/Szilard/nyu_v2_filter/dataset_plane/pcd_vox/

if [[ ! -z "$1" ]] 
then 
    default_dir=$1
fi
cd $default_dir

for filename in *.png; do
    cd $builddir
    # path, characters to delete from the end of the filename
    ./unifynames $default_dir$filename 38
done