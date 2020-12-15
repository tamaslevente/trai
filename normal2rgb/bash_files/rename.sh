#!/bin/bash
n=0
pre="ir_"
suf=".png"
for filename in *.png; do
    num=$pre$n$suf
    echo $num
    mv "$filename" $num
    ((n=n+1))
done