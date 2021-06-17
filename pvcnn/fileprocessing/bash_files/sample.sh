#!/bin/bash
cd $1
n=0
for filename in *.jpg; do
    echo $n
    if [[ $n -gt 0 ]]
    then 
       rm $filename
    fi
    ((n=n+1))
    if [[ $n -gt 2 ]]
    then
       ((n=0))
    fi
done