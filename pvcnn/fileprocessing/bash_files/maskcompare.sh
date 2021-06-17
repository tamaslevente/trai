#!bin/bash
builddir=$PWD'/../build/'
gtdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/maskgt/
preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/mask/
outdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/mask_diff/

if [[ ! -z "$1" ]] 
then 
    gtdir=$1
    if [[ ! -z "$2" ]] 
    then 
        preddir=$2
        if [[ ! -z "$3" ]] 
        then 
            outdir=$3
        fi
    fi
fi
cd $gtdir

declare -a arrGT
for file in *.png
do
    arrGT=("${arrGT[@]}" "$file")
done

cd $preddir
declare -a arrP
for file in *.png
do
    arrP=("${arrP[@]}" "$file")
done

cd $builddir

for i in "${!arrGT[@]}"; do
    gtfilename="${arrGT[i]}"
    predfilename="${arrP[i]}"
    ./maskcompare $gtdir $preddir $outdir $gtfilename $predfilename
done