#!bin/bash
builddir=$PWD'/../build/'
default_dir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pcndepth/ # path to input file directory

if [[ ! -z "$1" ]] 
then 
    default_dir=$1
fi
cd $default_dir

for filename in *.jpg; do
    cd $builddir
    ./jpg2png $default_dir$filename
done