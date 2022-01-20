

for f in *; do
    if [ -d "$f" ]; then
        # Will not run if no directories are available
        cd "$f"
        echo $f
        
        for g in *; do
        if [ -f "$g" ]; then
        # Will not run if no directories are available
        
        echo $g
        
        
        
		timeout 2 rosrun pcl_ros pcd_to_pointcloud $g /cloud_pcd:=/cloud_pcd_0 1
		
        fi
        done
        
        cd ..
    fi
       
done
