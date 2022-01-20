for i in ./*/;do

cd "$i";
echo $i;
echo '####'
pwd




for FILE in *;do 
    FILE2=${FILE//[a-z,.]};
    echo "${FILE2//[a-z,.]/}"
    mkdir $FILE2;
    mv $FILE $FILE2;
done

cd ..
done 
