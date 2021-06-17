#!/bin/bash
cd $1
n=0
mkdir "test"
mkdir "train"
for filename in *.png; do
    echo $n
    if [[ $n -gt 0 ]]
    then
       mv "$filename" "train/$filename"
    else
       mv "$filename" "test/$filename"
    fi
    ((n=n+1))
    if [[ $n -gt 2 ]]
    then
       ((n=0))
    fi
done
