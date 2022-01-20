#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float leaf_size,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud(points);
  vox_grid.filter(*downsampled_out);
}

void compute_surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float normal_radius,
                             pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

  // Use a FLANN-based KdTree to perform neighborhood searches
  norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));

  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch(normal_radius);

  // Set the input points
  norm_est.setInputCloud(points);

  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute(*normals_out);
}

void visualize_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr points,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr normal_points,
                       const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud(points, "points");
  viz.addPointCloud(normal_points, "normal_points");

  viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(normal_points, normals, 1, 0.01, "normals");

  // Give control over to the visualizer
  viz.spin();
}

int main(int argc, char **argv)
{
	int nr_volume=0;
  int nr_pointclouds=0;

  std::string string_pointcloud;
  std::ifstream MyReadFile2("../config_files/input_folder.txt");
  
  std::stringstream output_file;

  int file_nr=0;
  
	std::string line;
while (std::getline(MyReadFile2, string_pointcloud))
{

  int nr_final_pointcloud=0;

  fs::path path(string_pointcloud);
	for (auto& p : fs::directory_iterator(path))
	{
    nr_final_pointcloud++;
  }

	file_nr++;

  std::cout<<"FIle number:"<<file_nr<<'\n';

  
  
  for (auto& p : fs::directory_iterator(path))
	{
		
	std::stringstream ss;
	
    ss << p ;
    
   std::string str = ss.str();
   
   str.erase(std::remove(str.begin(), str.end(), '"'), str.end());
   
   std::cout<<str<<'\n';
	
  // Load data from pcd
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(str, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from "<<str
            << std::endl;

  
   
    
  
    
    nr_pointclouds++;
    
    std::string path_out="/home/alex-pop/Desktop/Doctorat/Side_projects/Volume_box_batch_processing/orthogonal-planes/Alex_pop_work/data_in/";
    
    std::string save_string=path_out;
    
    std::stringstream ss_pcd;
		ss_pcd << nr_pointclouds;
	std::string pcd_nr = ss_pcd.str();
    
    pcl::io::savePCDFileASCII(save_string +pcd_nr+ "_norm.pcd", *cloud);
    cout << "[*] Conversion finished!" << endl;
    
    
 
}

}

  return (0);
}
