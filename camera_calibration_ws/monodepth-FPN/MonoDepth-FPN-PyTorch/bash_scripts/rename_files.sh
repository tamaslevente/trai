#!/bin/bash
n=0
pre="ir_"
suf=".png"
files_in_folder=$(ls $1 | wc -l)
last_file=$((files_in_folder-1))
for i in $(seq 0 $last_file); do
    num=$1$n$suf
    filename="$1$i.png"
    if [ -e $filename ]; then
        echo $filename
        # mv "$filename" $num
    fi
    ((n=n+1))
done