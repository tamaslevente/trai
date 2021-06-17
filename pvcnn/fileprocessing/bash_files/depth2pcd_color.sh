#!bin/bash
builddir=$PWD'/../build/'
ddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/depthpred/
ndir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/normal/
pcddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/pcdpred/

if [[ ! -z "$1" ]] 
then 
    ddir=$1
fi
cd $ddir

declare -a arrD
for file in *.png
do
    arrD=("${arrD[@]}" "$file")
done

cd $ndir
declare -a arrN
for file in *.png
do
    arrN=("${arrN[@]}" "$file")
done

cd $builddir

for i in "${!arrD[@]}"; do
    dfilename="${arrD[i]}"
    nfilename="${arrN[i]}"
    ./depth2pcd_color $ddir $ndir $pcddir $dfilename $nfilename nyu
done