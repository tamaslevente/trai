#!bin/bash
cd $1
for filename in *.jpg; do
    cd "$2"
    echo "$1$filename" | ./jpg2png
    #rm "$1$filename"
done

