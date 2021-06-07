#!bin/bash
builddir=$PWD'/../build/'

# path, leaf size, meanK, StddevMulThresh
histo=histogram_tofnest_kitti.txt
start=0
end=1000
gtdir=/media/rambo/ssd2/Szilard/kitti/tofnest_selection/pcd_normals/
deltadir=/media/rambo/ssd2/Szilard/kitti/tofnest_selection/pcd_pred_delta/
gtending=.pcd
step=1

preddir=/media/rambo/ssd2/Szilard/kitti/tofnest_selection/pcd_pred/
predending=.pcd

# echo $preddir
cd $builddir
dont_save_delta_pcd=no
case $1 in
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

