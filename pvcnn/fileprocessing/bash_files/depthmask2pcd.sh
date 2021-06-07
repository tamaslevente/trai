#!bin/bash
builddir=$PWD'/../build/'
ddir=/media/rambo/ssd2/Szilard/toffilter_nyu/evaluation/
mdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/mask/
pcddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/pcdpredmask/

if [[ ! -z "$1" ]] 
then 
    ddir=$1
    if [[ ! -z "$2" ]] 
    then 
        mdir=$2
        if [[ ! -z "$3" ]] 
        then 
            pcddir=$3
        fi
    fi
fi
cd $ddir

declare -a arrD
for file in *.png
do
    arrD=("${arrD[@]}" "$file")
done

cd $mdir
declare -a arrM
for file in *.png
do
    arrM=("${arrM[@]}" "$file")
done

cd $builddir

for i in "${!arrD[@]}"; do
    dfilename="${arrD[i]}"
    mfilename="${arrM[i]}"
    ./depthmask2pcd $ddir $mdir $pcddir $dfilename $mfilename nyu
done