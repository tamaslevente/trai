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

int main(int argc, char **argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  std::string idir = argv[1];
  std::string odir = argv[2];
  std::string filename = argv[3];
  std::string leafname = argv[4] + 2;
  float leaf_size = atof(argv[4]);
  int MeanK = atoi(argv[5]);
  double StddevMulThresh = atof(argv[6]);

  char file_in[200];
  sprintf(file_in, "%s%s", idir.c_str(), filename.c_str());
  filename = filename.substr(0, filename.size() - 4);
  char file_out[200];
  sprintf(file_out, "%s%s_%s_filtered.pcd", odir.c_str(), filename.c_str(), leafname.c_str());
  std::cout << "Voxel filter for: " << filename.c_str() << std::endl;
  reader.read(file_in, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*cloud_filtered_voxel);
  std::cout << "PointCloud after filtering has: " << cloud_filtered_voxel->size() << " data points." << std::endl; //*

  pcl::PCDWriter writer;
  pcl::io::savePCDFile(file_out, *cloud_filtered_voxel, true);

  return (0);
}