#!bin/bash
builddir=$PWD'/../build/'
xyzdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/xyz/
ndir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/normal/
pcddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/working/pcd/

if [[ ! -z "$1" ]] 
then 
    xyzdir=$1
fi
cd $xyzdir

declare -a arrXYZ
for file in *.xyz
do
    arrXYZ=("${arrXYZ[@]}" "$file")
done

cd $ndir
declare -a arrN
for file in *.normals
do
    arrN=("${arrN[@]}" "$file")
done

cd $builddir

for i in "${!arrXYZ[@]}"; do
    xyzfilename="${arrXYZ[i]}"
    nfilename="${arrN[i]}"
    ./xyzn2pcdnormal $xyzdir $ndir $pcddir $xyzfilename $nfilename
done