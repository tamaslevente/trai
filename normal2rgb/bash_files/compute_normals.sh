#!bin/bash
cd $1
for filename in *filtered.pcd; do
    cd "$2"
    echo "$1$filename" | ./compute_normals
done