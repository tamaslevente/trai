#!bin/bash
builddir=$PWD'/../build/'
ddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/predv48/predictions_n00/
mdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/mask/
meanK=50
stddevMulThresh=1.0

if [[ ! -z "$1" ]] 
then 
    ddir=$1
fi
cd $ddir

for filename in *.png; do
    cd $builddir
    # path,cameratype
    ./depth_outlier_mask $ddir $mdir $filename nyu $meanK $stddevMulThresh
done