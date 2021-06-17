#!bin/bash
# preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/dataset_plane/depth3/
preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/depth3_/
workdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/depthpred/

if [[ ! -z "$1" ]] 
then 
    preddir=$1
fi
cd $preddir

for filename in *pred.png; do
    mv $preddir$filename $workdir$filename
done