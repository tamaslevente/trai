#!bin/bash
builddir=$PWD'/../build/'

# path, leaf size, meanK, StddevMulThresh
histo=histogram_$1.txt
start=0
end=1449
gtdir=/media/rambo/ssd2/Szilard/toffilter_nyu/evaluation/$2gt/

case $1 in
    base) # original noisy data
    echo BASE
        preddir=/media/rambo/ssd2/Szilard/toffilter_nyu/evaluation/$2/
        ;;
    own) # toffilter prediction
        echo ToFFilter
        preddir=/media/rambo/ssd2/Szilard/toffilter_nyu/evaluation/$2_pred/
        ;;
    ownmask) # toffilter prediction with mask
        echo ToFFilterMask
        preddir=/media/rambo/ssd2/Szilard/toffilter_nyu/evaluation/pcd_pred_mask/
        ;;
    *) # toffilter
        echo Undefined, ToFFilter
        preddir=/media/rambo/ssd2/Szilard/toffilter_nyu/evaluation/$2_pred/
        ;;    
esac
# echo $preddir
cd $builddir
step=1
case $2 in
    depth)
        echo depth
        gtending=.png
        predending=.png
        ./pw_depthcompare $gtdir $preddir $gtending $predending $start $end $step
        ;;
    pcd)
        echo pcd
        predending=.pcd
        ./pw_pcdcompare $gtdir $preddir $gtending $predending $start $end $step nyu 480 640
        ;;
    *)
        predending=.png
        gtending=.png
        ./pw_depthcompare $gtdir $preddir $gtending $predending $start $end $step
        ;;
esac