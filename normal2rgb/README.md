This is a project which has as input the pointclouds as pcd files.
The program applies a voxel filter of size 0.005 m (by default, can be set up in src/voxel_filter.cpp. 
Set leaf_size and filename at the end of the file) upon the point cloud.
It computes normals for every point. (The pcd files can be converted to xyz format to be fed into NestiNet or NINormals function to generate normals).
The normals are stored as PointNormals type (x,y,z,normal_x, normal_y, normal_z, curvature).
These normals are then converted to XYZRGB pointclouds, by converting the 3 dimensional direction of the normal vectors into the 3 color channels.
Then the script creates png images from the  pointclouds.

How to convert your pcd files into images where the color decodes the direction of the normals:

mkdir build

cd build

cmake ..

make

Then go to the baash_files directory, and run:

bash pcd2image.sh <path_to_your_pcd_files> <path_to_the_build_directory>
