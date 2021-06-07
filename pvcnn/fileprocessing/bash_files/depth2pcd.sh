#!bin/bash
builddir=$PWD'/../build/'
ddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/depthpred/
pcddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/pcdpred/

if [[ ! -z "$1" ]] 
then 
    ddir=$1
    if [[ ! -z "$2" ]] 
    then 
        pcddir=$2
    fi
fi
cd $ddir

for filename in *.png; do
    cd $builddir
    # path
    ./depth2pcd $ddir $pcddir $filename kitti
done