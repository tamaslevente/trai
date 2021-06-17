#!bin/bash
builddir=$PWD'/../build/'

# path, leaf size, meanK, StddevMulThresh
histo=histogram_own_v48m_00.txt
noise=n00
start=0
end=1449
gtdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pcn/pcn_n00/
gtending=.pcd

case $1 in
    base) # original noisy data
    echo BASE
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pcdnoise_$noise/
        predending=.pcd
        ;;
    own) # toffilter prediction
        echo ToFFilter
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/pcdpred_$noise/
        predending=_pred.pcd
        ;;
    sor) # statistical outlier removal
        echo StatisticalOutlierRemoval
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pclsor/$noise'_sor/'
        predending=_sor.pcd
        ;;
    pcn) # pointcleannet
        echo PointCleanNet
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pcn/pcn_$noise/
        predending=.pcd
        ;;
    matlab) #matlab denoiser
        echo Matlab
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/matlab_pred/matlab_$noise/
        predending=_matlab.pcd
        ;;
    ddd) # deep depth denoising
        echo DeepDepthDenoising
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/ddd/ddd_$noise/
        predending=_ddd.pcd
        ;;
    ownmask) # toffilter prediction with mask
        echo ToFFilterMask
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/pcdmasked/$noise/
        predending=_mask.pcd
        ;;
    *) # toffilter
        echo Undefined, ToFFilter
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/own/pcdpred_$noise/
        predending=.pcd
        ;;    
esac
# echo $preddir
cd $builddir
step=1
./comparepcd $histo $gtdir $preddir $gtending $predending $start $end $step
