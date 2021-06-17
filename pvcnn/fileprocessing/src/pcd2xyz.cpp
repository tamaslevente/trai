#include <iostream>
#include <fstream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string idir = argv[1];
  std::string odir = argv[2];
  std::string filename = argv[3];
  char file_in[200];
  sprintf(file_in, "%s%s", idir.c_str(), filename.c_str());
  filename = filename.substr(0, filename.size() - 3);
  char file_out[200];
  sprintf(file_out, "%s%sxyz", odir.c_str(), filename.c_str());

  pcl::PCDReader reader;
  reader.read(file_in, *cloud);
  ofstream myfile;
  myfile.open(file_out);
  for (int i = 0; i < cloud->size(); i++)
  {
    myfile << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\n";
  }
  //myfile << "Writing this to a file.\n";
  myfile.close();
  return 0;
}