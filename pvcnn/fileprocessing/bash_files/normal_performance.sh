#!bin/bash
builddir=$PWD'/../build/'

# path, leaf size, meanK, StddevMulThresh
histo=histogram_tofnest.txt
start=0
end=1449
gtdir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pcn/
deltadir=/media/rambo/ssd2/Szilard/nyu_v2_tofnest/tofnest/pcdpred_delta/
gtending=.pcd
step=1

case $1 in
    own) # ToFNest
        echo ToFNest
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_tofnest/tofnest/pcdpred/
        predending=_pred.pcd
        ;;
    nesti) # Nesti-Net
        echo Nesti-Net
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/nestigen/
        predending=_nestinormals.pcd
        ;;
    pcpss) # PCPNet single scale
        echo PCPNet_ss
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pcpnetgen/single_scale_normal/
        predending=_pcpnetnormals.pcd
        ;;
    pcpms) #PCPNet multi scale
        echo PCPNet_ms
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pcpnetgen/multi_scale_normal/
        predending=_pcpnetnormals.pcd
        ;;
    pcl) # Point CLoud Normals
        echo PCLNORMALS
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/pclnormals/pclnormals/
        predending=_pclouds_normals_pclnormals.pcd
        ;;
    hough) # Hough Normals
        echo HOUGHNORMALS
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_filter/comparison/houghgen/
        predending=_houghnormals.pcd
        ;;
    *) # toffilter
        echo Undefined, ToFNest
        preddir=/media/rambo/ssd2/Szilard/nyu_v2_tofnest/tofnest/pcdpred/
        predending=_pred.pcd
        ;;    
esac
# echo $preddir
cd $builddir
dont_save_delta_pcd=no
case $2 in
    quality)
        ./normal_performance_quality $histo $gtdir $preddir $gtending $predending $dont_save_delta_pcd $start $end $step
        ;;
    quality_delta)
        ./normal_performance_quality $histo $gtdir $preddir $gtending $predending $deltadir $start $end $step
        ;;
    angle)
        ./normal_performance_angle $histo $gtdir $preddir $gtending $predending $start $end $step
        ;;
    distribution)
        ./normal_performance_distribution $gtdir $preddir $gtending $predending $start $end $step
        ;;
    *)
        ./normal_performance_quality $histo $gtdir $preddir $gtending $predending $deltadir $start $end $step
        ;;
esac

