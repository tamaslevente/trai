#!bin/bash
builddir=$PWD'/../build/'
input_dir=/media/rambo/ssd2/Szilard/nyu_v2_filter/pcn_or/info/
output_dir=/media/rambo/ssd2/Szilard/nyu_v2_filter/pcn_or/pcd/

if [[ ! -z "$1" ]] 
then 
    input_dir=$1
    if [[ ! -z "$2" ]] 
    then 
        output_dir=$2
    fi
fi
cd $input_dir

for filename in *.xyz; do
    cd $builddir
    ./xyz2pcd $input_dir $output_dir $filename
done