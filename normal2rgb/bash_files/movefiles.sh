#!/bin/bash
cd $1
mkdir "pcd"
for filename in *.pcd; do
    file="pcd/"$filename
    mv "$filename" "$file"
done

mkdir "depthir"
for filename in *depthir.png; do
    file="depthir/"$filename
    mv "$filename" "$file"
done

mkdir "depth"
for filename in *depth.png; do
    file="depth/"$filename
    mv "$filename" "$file"
done

mkdir "ir"
for filename in *ir.png; do
    file="ir/"$filename
    mv "$filename" "$file"
done

mkdir "rgb"
for filename in *rgb.png; do
    file="rgb/"$filename
    mv "$filename" "$file"
done

mkdir "normalimages"
