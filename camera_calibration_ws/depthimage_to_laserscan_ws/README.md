# Some configurations in order to use depthimage_to_laserscan while not wasting too much time

## ADITOF Camera

Assuming that you put the launch files from this repo into your catkin_ws/src/aditof_roscpp/lauch and depth_image_format.py in your catkin_ws, you can follow the next steps:

1. Start the camera (maybe you will need root privileges first (`sudo su`)): 
`roslaunch aditof_roscpp camera_node.launch`

2. Rectify the image (don't know if it's absolutely necessary, but it doesn't hurt): 
`ROS_NAMESPACE=/aditof_roscpp rosrun image_proc image_proc /aditof_roscpp/camera_info:=/aditof_roscpp/aditof_camera_info /aditof_roscpp/image_raw:=/aditof_roscpp/aditof_depth` 

3. Run the script which transforms from `mono16` (camera's raw images) to `16UC1` (the format accepted by the depthimage_to_laserscan package (or `32FC1`)): 
`python depth_image_format.py`

4. Start the laser scan: 
`roslaunch aditof_roscpp depth_to_laser.launch`

5. Start the last node (aka The Savior), which will overlap the point cloud and the laser scan axes: 
`roslaunch aditof_roscpp tf_important_for_laser.launch`

## PICO ZENSE Camera 

With this camera, we had some troubles when using this package (segmentation fault, without any other warning) and that was because our camera_info didn't contained the R matrix, and without it couldn't make the conversion from depth to laser, so first make sure you have this matrix. 

Making the same assumptions as above, you should follow the next steps:
1. Start the camera: 
`roslaunch pico_zense_camera pz_camera.launch`

2. Start the pointcloud node (maybe, you might need to install this: `sudo apt install ros-melodic-depth-image-proc`):
`roslaunch pico_zense_camera camera_node_depth_image_proc.launch`

3. Start the laser scan: 
`roslaunch pico_zense_camera depth_laser_scan.launch`

4. Run The Savior: 
`roslaunch pico_zense_camera tf_important_for_laser.launch`

5. And... that's it, you can open `rviz` to visualise this beauty.

__** Disclaimer__: These configurations where tested only for pico zense and aditof cameras. 