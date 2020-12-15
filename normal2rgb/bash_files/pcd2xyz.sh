#!bin/bash
cd $1
for filename in *filtered.pcd; do
    cd "/home/szilard/projects/normalrgb/build"
    echo "$1$filename" | ./pcd2xyz
done

