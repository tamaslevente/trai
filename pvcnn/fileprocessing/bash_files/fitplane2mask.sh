#!bin/bash
builddir=$PWD'/../build/'
ddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/depthpred/
mdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/mask/

if [[ ! -z "$1" ]] 
then 
    ddir=$1
fi
cd $ddir

for filename in *.png; do
    cd $builddir
    # depth directory, mask directory, filename
    ./fitplane2mask $ddir $mdir $filename nyu
done