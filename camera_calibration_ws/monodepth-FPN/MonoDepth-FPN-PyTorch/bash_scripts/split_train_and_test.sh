#!/bin/bash
# this script will split your data into a training set a test (aka validation (it should've been from the beginning named validation, but... now, I'm stuck with this name, sorry!)) set
# also you need to specify as a command line argument the folder with a "/" at the end

n=0
# pre="ir_"
suf=".png"

mkdir -p $1train
mkdir -p $1test

train=$1train
test=$1test

for filename in $1*.png; do
    if ((n%3==0))
    then
        echo $test
        # echo $filename
        mv "$filename" $test
    else
        echo $train
        # echo $filename
        mv "$filename" $train
    fi
    ((n=n+1))
    echo $n
done