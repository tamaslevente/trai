#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
  printf("Doing my job! Please don't peek, just... WAIT!\n");
  pcl::PointCloud<pcl::PointXYZ> cloud;


  int pointcloud_size = 2500000;

  // // multiP1
  // float x_min_bound = -0.6;
  // float x_max_bound = 0.6;
  // float y_min_bound = -0.6;
  // float y_max_bound = 0.6;
  // float z_min_bound = -0.6;
  // float z_max_bound = 0.6;

  // // multiP2
  // float x_min_bound = -0.7;
  // float x_max_bound = 0.7;
  // float y_min_bound = -0.7;
  // float y_max_bound = 0.7;
  // float z_min_bound = -0.7;
  // float z_max_bound = 0.7;

  // // multiP3
  // float x_min_bound = -1.2;
  // float x_max_bound = 1.2;
  // float y_min_bound = -1.2;
  // float y_max_bound = 1.2;
  // float z_min_bound = -1.2;
  // float z_max_bound = 1.2;

  // // multiP4
  // float x_min_bound = -5;
  // float x_max_bound = 5;
  // float y_min_bound = -5;
  // float y_max_bound = 5;
  // float z_min_bound = -1;
  // float z_max_bound = 10;

  // // multiP5
  // float x_min_bound = -5;
  // float x_max_bound = 5;
  // float y_min_bound = -5;
  // float y_max_bound = 5;
  // float z_min_bound = -1;
  // float z_max_bound = 10;

  // multiP6
  float x_min_bound = -5;
  float x_max_bound = 5;
  float y_min_bound = -5;
  float y_max_bound = 5;
  float z_min_bound = -1;
  float z_max_bound = 10;


  double x, y, z;
  for (int i = 0; i < pointcloud_size; i++)
  {
    x = (x_max_bound - x_min_bound) * ((rand() - 0) / (RAND_MAX + 1.0f - 0)) + x_min_bound;
    y = (y_max_bound - y_min_bound) * ((rand() - 0) / (RAND_MAX + 1.0f - 0)) + y_min_bound;
    z = (z_max_bound - z_min_bound) * ((rand() - 0) / (RAND_MAX + 1.0f - 0)) + z_min_bound;
    cloud.push_back(pcl::PointXYZ(x,y,z));
  }

  // }

  pcl::io::savePCDFileASCII("my_full_custom_pcdmultiP6.pcd", cloud);
  std::cerr << "Saved " << cloud.size() << " data points to some *.pcd." << std::endl;

  return (0);
}
