#!/bin/bash
n=0
# pre="ir_"
suf=".png"

mkdir -p $1train
mkdir -p $1test

train=$1train
test=$1test

for filename in $1/*.png; do
    if ((n%4==0))
    then
        echo $test
        mv "$filename" $test
    else
        mv "$filename" $train
        echo $train
    fi
    ((n=n+1))
    echo $n
done