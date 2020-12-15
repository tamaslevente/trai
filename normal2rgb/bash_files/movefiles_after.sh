#!/bin/bash
cd $1
mkdir "pcd"
for filename in *pcd.pcd; do
    file="pcd/"$filename
    mv "$filename" "$file"
done

mkdir "filtered"
for filename in *filtered.pcd; do
    file="filtered/"$filename
    mv "$filename" "$file"
done

mkdir "pclnormals"
for filename in *pclnormals.pcd; do
    file="pclnormals/"$filename
    mv "$filename" "$file"
done

mkdir "n2rgb"
for filename in *n2rgb.pcd; do
    file="n2rgb/"$filename
    mv "$filename" "$file"
done

for filename in *rgb.png; do
    file="../normalimages/"$filename
    mv "$filename" "$file"
done
