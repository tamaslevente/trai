#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <experimental/filesystem>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>



#include <pcl/filters/extract_indices.h>




#define PI 3.14159265




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



 void planar_segmenting_single_time(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented,
                                     pcl::ModelCoefficients::Ptr coefficients)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    // Segment dominant plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_segmented);

    if (inliers->indices.size() == 0)
    {

      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      //ok2 = 0;
    }
  }


   void planar_segmenting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                         float Coeficients[3][4],
                         pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                         int t,
                         bool &ok2)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    // Segment dominant plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_segmented);

    if (inliers->indices.size() == 0)
    {

      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      ok2 = 0;
    }

    else
    {

      Coeficients[t - 1][0] = coefficients->values[0];
      Coeficients[t - 1][1] = coefficients->values[1];
      Coeficients[t - 1][2] = coefficients->values[2];
      Coeficients[t - 1][3] = coefficients->values[3];

      *cloud_final += *cloud_segmented;

      all_planes[t] = cloud_segmented;

      ok2 = 1;
    }
  }

  void euclidean_segmenting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f,
                            bool &ok2)
  {

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int i = 0, nr_points = (int)cloud_filtered->points.size();

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      //////
      ///////////
      //////////
      // TREBUIE FUNCTIE DE OPRIRE
      /////////////
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Write the planar inliers to disk
    extract.filter(*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f; //     HERE IS THE CLOUD FILTERED EXTRACTED

    if (cloud_filtered->size() != 0)
    {
      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(0.02); // 2cm
      ec.setMinClusterSize(100);
      ec.setMaxClusterSize(25000);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_filtered);
      ec.extract(cluster_indices);

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
          cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cloud = cloud_cluster;
        ok2 = 1;
      }

      ////////////////////////////////////
    }
    else
    {
      ok2 = 0;
    }
  }

  void check_passthrough_limits_x_min(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                                float threshold_x,
                                float threshold_y,
                                float threshold_z,
                                float z_lower_limit,
                                float z_upper_limit,
                                float y_lower_limit,
                                float y_upper_limit,
                                float x_lower_limit,
                                float x_upper_limit,
                                int minimum_nr_points,
                                int &nr_points,
                                pcl::PointCloud<pcl::PointXYZ> &final_pointcloud)
  {
    
   int nr_puncte[3][2];

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        nr_puncte[i][j] = 0;
      }
    }

    for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {

      if (cloud_final->points[nIndex].x < x_lower_limit + threshold_x)
      {
        nr_puncte[0][0] = nr_puncte[0][0] + 1;
      
      }

    }

    if (nr_puncte[0][0] > minimum_nr_points)
    {
      //std::cout << "Object cut X min. Move towards X_min" << '\n';

      for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
      {
          if (cloud_final->points[nIndex].x < x_lower_limit + threshold_x)
          {
            int N = final_pointcloud.width;

            final_pointcloud.width = final_pointcloud.width + 1;
            final_pointcloud.is_dense = false;
             final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

            final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
            final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
            final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
          }
      }

      nr_points= nr_puncte[0][0];

     // std::cout<<"Puncte X min:"<<nr_points<<'\n';
    }

  }

  void check_passthrough_limits_x_max(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                                float threshold_x,
                                float threshold_y,
                                float threshold_z,
                                float z_lower_limit,
                                float z_upper_limit,
                                float y_lower_limit,
                                float y_upper_limit,
                                float x_lower_limit,
                                float x_upper_limit,
                                int minimum_nr_points,
                                int &nr_points,
                                pcl::PointCloud<pcl::PointXYZ> &final_pointcloud)
  {
    
   int nr_puncte[3][2];

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        nr_puncte[i][j] = 0;
      }
    }

    for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {


      if (cloud_final->points[nIndex].x > x_upper_limit - threshold_x)
      {
        nr_puncte[0][1] = nr_puncte[0][1] + 1;
        
      }


    }


    if (nr_puncte[0][1] > minimum_nr_points)
    {
      //std::cout << "Object cut X max. Move towards X_max" << '\n';

      for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
      {
         if (cloud_final->points[nIndex].x > x_upper_limit - threshold_x)
          {
            int N = final_pointcloud.width;

            final_pointcloud.width = final_pointcloud.width + 1;
            final_pointcloud.is_dense = false;
             final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

            final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
            final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
            final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
          }
      }
       nr_points= nr_puncte[0][1];

      // std::cout<<"Puncte X max:"<<nr_points<<'\n';
    }
  }

  void check_passthrough_limits_y_min(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                                float threshold_x,
                                float threshold_y,
                                float threshold_z,
                                float z_lower_limit,
                                float z_upper_limit,
                                float y_lower_limit,
                                float y_upper_limit,
                                float x_lower_limit,
                                float x_upper_limit,
                                int minimum_nr_points,
                                int &nr_points,
                                pcl::PointCloud<pcl::PointXYZ> &final_pointcloud)
  {
    
    int nr_puncte[3][2];

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        nr_puncte[i][j] = 0;
      }
    }

    for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {

      if (cloud_final->points[nIndex].y < y_lower_limit + threshold_y)
      {
        nr_puncte[1][0] = nr_puncte[1][0] + 1;
      
      }
    }

    if (nr_puncte[1][0] > minimum_nr_points)
    {
      //std::cout << "Object cut Y min. Move towards Y_min" << '\n';

      for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
      {
          if (cloud_final->points[nIndex].y < y_lower_limit + threshold_y)
          {
            int N = final_pointcloud.width;

            final_pointcloud.width = final_pointcloud.width + 1;
            final_pointcloud.is_dense = false;
             final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

            final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
            final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
            final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
          }
      }
       nr_points= nr_puncte[1][0];

       //std::cout<<"Puncte Y min:"<<nr_points<<'\n';
    }
  }

void check_passthrough_limits_y_max(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                                float threshold_x,
                                float threshold_y,
                                float threshold_z,
                                float z_lower_limit,
                                float z_upper_limit,
                                float y_lower_limit,
                                float y_upper_limit,
                                float x_lower_limit,
                                float x_upper_limit,
                                int minimum_nr_points,
                                int &nr_points,
                                pcl::PointCloud<pcl::PointXYZ> &final_pointcloud)
  {
    
    int nr_puncte[3][2];

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        nr_puncte[i][j] = 0;
      }
    }

    for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {

      if (cloud_final->points[nIndex].y > y_upper_limit - threshold_y)
      {
        nr_puncte[1][1] = nr_puncte[1][1] + 1;
        
      }


    }

    if (nr_puncte[1][1] > minimum_nr_points)
    {
      //std::cout << "Object cut Y max. Move towards Y_max" << '\n';

      for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
      {
         if (cloud_final->points[nIndex].y > y_upper_limit - threshold_y)
          {
            int N = final_pointcloud.width;

            final_pointcloud.width = final_pointcloud.width + 1;
            final_pointcloud.is_dense = false;
             final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

            final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
            final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
            final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
          }
      }
       nr_points= nr_puncte[1][1];

       //std::cout<<"Puncte Y max:"<<nr_points<<'\n';
    }

  }

float score_passthrough_side(int p,pcl::PointCloud<pcl::PointXYZ> cloud_proximitate, int limit_number)
    {
      if (p>=2)
      {
          if (cloud_proximitate.size()>limit_number)
           {
             //return(1 / cloud_proximitate.size()) ;
             return(cloud_proximitate.size());
           }
          else
          {
            return(cloud_proximitate.size());
          }
      }
      else
      {
        return(0);
      }
    }

void compute_angle( pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],
                                    float Coeficients[3][4],
                                    int j,
                                    float limit_angle,
                                    float &cos_angle,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr wrong_angle_plane)
  {

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);

    coefficients->values[0] = Coeficients[j - 1][0];
    coefficients->values[1] = Coeficients[j - 1][1];
    coefficients->values[2] = Coeficients[j - 1][2];
    coefficients->values[3] = Coeficients[j - 1][3];

   

      float length = sqrt ( (coefficients->values[0])*(coefficients->values[0])+
                            (coefficients->values[1])*(coefficients->values[1])+
                            (coefficients->values[2])*(coefficients->values[2]));
      float cosine_value= coefficients->values[2] * 1 / length;

      if (abs(cosine_value) < cos(  (limit_angle)*(PI) / 180 )   )
      {
          *wrong_angle_plane +=  *(all_planes[j]);
          //std::cout<<abs(cosine_value)<<" < "<<cos((limit_angle)*(PI)/180)<<'\n';

          cos_angle=abs(cosine_value);
      }

      
      

  } 

  float score_angle(int p,int nr_plane, float cos_angle,float cos_angle_limit)
    {
      if (p>nr_plane)
      {
          if (cos_angle<cos_angle_limit)
           {
             return(cos_angle) ;
           }
          else
          {
            //return(1);
            return(cos_angle);
          }
      }
      else
      {
        return(0);
      }
    }

int main(int argc, char **argv)
{
		float leaf_size=0.01;
  
  float z_lower_limit=-6;
	float z_upper_limit=50;
	
	float x_lower_limit=-1;
	float x_upper_limit=1;
	
	float y_lower_limit=-1;
	float y_upper_limit=1;
	
	

	int nr_minim_puncte=500;
  int nr_minim_puncte_plan=100;

  float threshold_distance_x=0.02;
  float threshold_distance_y=0.02;
  float threshold_distance_z=0.02;

  int min_nr_pts_pf_check=0;

  float angle_threshold=45;
	
	int nr_volume=0;
  int nr_pointclouds=0;

  
  
  //std::cout << "Voxel leaf size=";
  //std::cin>>leaf_size;
  
  

  std::string string_pointcloud;
  std::ifstream input_folder("../config_files/input_folder.txt");
  
  std::stringstream output_file;

  int file_nr=0;
  
	std::string line;
while (std::getline(input_folder, string_pointcloud))
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
            
            
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointXYZ cloud_point;         
            
   for (int i = 0; i < cloud->size(); i++)
    {if( (isnan(cloud->points[i].x)==0) && 
          (isnan(cloud->points[i].y)==0) && 
          (isnan(cloud->points[i].z)==0)){
      cloud_point.x = cloud->points[i].x;
      cloud_point.y = cloud->points[i].y;
      cloud_point.z = cloud->points[i].z;
      cloud_2->points.push_back(cloud_point);
  }
    }      
    
    cloud_2->width = cloud_2->points.size();
    cout << "Number of points:" << cloud_2->width << endl;
    cloud_2->height = 1;
    cloud_2->points.resize(cloud_2->width * cloud_2->height);
    cloud_2->is_dense = false;
            

  // Point Clouds to hold output
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	

    downsample(cloud_2,leaf_size,downsampled);
 
    pcl::PassThrough<pcl::PointXYZ> pt1_;
    pcl::PassThrough<pcl::PointXYZ> pt2_;
    pcl::PassThrough<pcl::PointXYZ> pt3_;
    
    pt1_.setFilterFieldName ("z");
    pt1_.setFilterLimits (z_lower_limit, z_upper_limit);
    
    pt2_.setFilterFieldName ("x");
    pt2_.setFilterLimits (x_lower_limit, x_upper_limit);
    

    pt3_.setFilterFieldName ("y");
    pt3_.setFilterLimits (y_lower_limit, y_upper_limit);
 
	  pt1_.setInputCloud(downsampled);
    pcl::PointCloud<pcl::PointXYZ> cloud_out1; 
    pt1_.filter(cloud_out1);
    

    
    
    
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_2(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_in_2 = cloud_out1;
 
 
	pt2_.setInputCloud(cloud_in_2);
    pcl::PointCloud<pcl::PointXYZ> cloud_out2; 
    pt2_.filter(cloud_out2);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_3(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_in_3 = cloud_out2;
 
	pt3_.setInputCloud(cloud_in_3);
    pcl::PointCloud<pcl::PointXYZ> cloud_out3; 
    pt3_.filter(cloud_out3);
    
  
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pf(new pcl::PointCloud<pcl::PointXYZ>); 
  
  *cloud_pf=cloud_out3;

    bool ok = 1;
    bool ok2 = 1;
    int nr_planes;

    float Coeficients[3][4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients_floor(new pcl::ModelCoefficients);
    Eigen::Vector3f normal_floor;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_euclidean(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pf_backup(new pcl::PointCloud<pcl::PointXYZ>);

    *cloud_pf_backup=*cloud_pf;

    pcl::PointCloud<pcl::PointXYZ> cloud_proximitate;

    cloud_proximitate.width = 0;
    cloud_proximitate.height = 1;
    cloud_proximitate.is_dense = false;
    cloud_proximitate.resize(cloud_proximitate.width * cloud_proximitate.height);

    pcl::PointCloud<pcl::PointXYZ> cloud_proximitate_x_min;
    cloud_proximitate_x_min.width = 0;
    cloud_proximitate_x_min.height = 1;
    cloud_proximitate_x_min.is_dense = false;
    cloud_proximitate_x_min.resize(cloud_proximitate_x_min.width * cloud_proximitate_x_min.height);

    pcl::PointCloud<pcl::PointXYZ> cloud_proximitate_x_max;
    cloud_proximitate_x_max.width = 0;
    cloud_proximitate_x_max.height = 1;
    cloud_proximitate_x_max.is_dense = false;
    cloud_proximitate_x_max.resize(cloud_proximitate_x_max.width * cloud_proximitate_x_max.height);

    pcl::PointCloud<pcl::PointXYZ> cloud_proximitate_y_min;
    cloud_proximitate_y_min.width = 0;
    cloud_proximitate_y_min.height = 1;
    cloud_proximitate_y_min.is_dense = false;
    cloud_proximitate_y_min.resize(cloud_proximitate_y_min.width * cloud_proximitate_y_min.height);

    pcl::PointCloud<pcl::PointXYZ> cloud_proximitate_y_max;
    cloud_proximitate_y_max.width = 0;
    cloud_proximitate_y_max.height = 1;
    cloud_proximitate_y_max.is_dense = false;
    cloud_proximitate_y_max.resize(cloud_proximitate_y_max.width * cloud_proximitate_y_max.height);

    int x_min_nr_points;
    int x_max_nr_points;
    int y_min_nr_points;
    int y_max_nr_points;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_wrong_angle(new pcl::PointCloud<pcl::PointXYZ>);

    int score1;
    float score_plane1;
    float score_plane2;
    float score_plane3;

    float cos_angle_u1=1;
    float cos_angle_u2=1;
    float cos_angle_u3=1;



  
  
  if (cloud_pf->size() < nr_minim_puncte)
    {
      ok = 0;
      nr_planes = 0; // NO PLANE
    }
    else
    {
      planar_segmenting_single_time(cloud_pf, cloud_floor, coefficients_floor);

      normal_floor << coefficients_floor->values[0], coefficients_floor->values[1], coefficients_floor->values[2];
      if (coefficients_floor->values[3] < 0)
      {
        normal_floor *= -1;
      }
      nr_planes = 1;
    }



   for (int t =1 ; (t < 4) && ok; t++)
    {

      euclidean_segmenting(cloud_pf, cloud_euclidean, ok2);

      if (cloud_euclidean->size() < nr_minim_puncte_plan)
      {
        ok = 0; //No more planes to cut out
      }

      if (ok2 != 0)
      {
        planar_segmenting(cloud_euclidean, Coeficients, all_planes, cloud_final, t, ok2);

        cloud_pf = cloud_euclidean; // Cloud is now the extracted pointcloud

        if (cloud_pf->size() < nr_minim_puncte)
        {
          ok = 0;
        }

        nr_planes = t + 1;
      }
    } 

     if (nr_planes >= 2)
    {
      check_passthrough_limits_x_min(cloud_final,
                               threshold_distance_x,
                               threshold_distance_y,
                               threshold_distance_z,
                               z_lower_limit,
                               z_upper_limit,
                               y_lower_limit,
                               y_upper_limit,
                               x_lower_limit,
                               x_upper_limit,
                               min_nr_pts_pf_check,
                               x_min_nr_points,
                               cloud_proximitate_x_min);

      cloud_proximitate=cloud_proximitate+cloud_proximitate_x_min;

      check_passthrough_limits_x_max(cloud_final,
                               threshold_distance_x,
                               threshold_distance_y,
                               threshold_distance_z,
                               z_lower_limit,
                               z_upper_limit,
                               y_lower_limit,
                               y_upper_limit,
                               x_lower_limit,
                               x_upper_limit,
                               min_nr_pts_pf_check,
                               x_max_nr_points,
                               cloud_proximitate_x_max);

      cloud_proximitate+=cloud_proximitate_x_max;

      check_passthrough_limits_y_min(cloud_final,
                               threshold_distance_x,
                               threshold_distance_y,
                               threshold_distance_z,
                               z_lower_limit,
                               z_upper_limit,
                               y_lower_limit,
                               y_upper_limit,
                               x_lower_limit,
                               x_upper_limit,
                               min_nr_pts_pf_check,
                               y_min_nr_points,
                               cloud_proximitate_y_min);

      cloud_proximitate+=cloud_proximitate_y_min;

      check_passthrough_limits_y_max(cloud_final,
                               threshold_distance_x,
                               threshold_distance_y,
                               threshold_distance_z,
                               z_lower_limit,
                               z_upper_limit,
                               y_lower_limit,
                               y_upper_limit,
                               x_lower_limit,
                               x_upper_limit,
                               min_nr_pts_pf_check,
                               y_max_nr_points,
                               cloud_proximitate_y_max);

      cloud_proximitate+=cloud_proximitate_y_max;

    }

    if(nr_planes==4)
    {

    

    compute_angle(all_planes,
                   Coeficients,
                   1,
                  angle_threshold,
                  cos_angle_u1,
                  cloud_wrong_angle);

      compute_angle(all_planes,
                   Coeficients,
                   2,
                  angle_threshold,
                  cos_angle_u2,
                  cloud_wrong_angle);
      
      compute_angle(all_planes,
                   Coeficients,
                   3,
                  angle_threshold,
                  cos_angle_u3,
                  cloud_wrong_angle);

      


      }
      else{
        if(nr_planes==3)
        {
          compute_angle(all_planes,
                   Coeficients,
                   1,
                  angle_threshold,
                  cos_angle_u1,
                  cloud_wrong_angle);

          compute_angle(all_planes,
                   Coeficients,
                   2,
                  angle_threshold,
                  cos_angle_u2,
                  cloud_wrong_angle);
        }
        else{
          if(nr_planes==2)
          {
             compute_angle(all_planes,
                   Coeficients,
                   1,
                  angle_threshold,
                  cos_angle_u1,
                  cloud_wrong_angle);
          }
        }
      }



    score1=score_passthrough_side(nr_planes,cloud_proximitate_x_min,min_nr_pts_pf_check)+
          score_passthrough_side(nr_planes,cloud_proximitate_x_max,min_nr_pts_pf_check)+
          score_passthrough_side(nr_planes,cloud_proximitate_y_min,min_nr_pts_pf_check)+
          score_passthrough_side(nr_planes,cloud_proximitate_y_max,min_nr_pts_pf_check);
    //std::cout<<"Score 1:"<<score1<<'\n';

    score_plane1=score_angle(nr_planes,1,cos_angle_u1,angle_threshold);
    score_plane2=score_angle(nr_planes,2,cos_angle_u2,angle_threshold);
    score_plane3=score_angle(nr_planes,3,cos_angle_u3,angle_threshold);


 
    nr_pointclouds++;
    
   
    std::string path_out="/home/alex-pop/Desktop/BAckups/Alex_pop_work/data_out/";
    
    std::string save_string=path_out;
    
    std::stringstream ss_pcd;
		ss_pcd << nr_pointclouds;
	  std::string pcd_nr = ss_pcd.str();

    std::string out_score_name="PCL";
    std::string output_file_score_2="/home/alex-pop/Desktop/BAckups/Alex_pop_work/data_out/Score/Score_test_";

    std::string names_path="/home/alex-pop/Desktop/BAckups/Alex_pop_work/data_out/Score/names.txt";

    output_file_score_2=output_file_score_2+out_score_name.c_str()+".csv";

    std::ofstream log_score(output_file_score_2, std::ios_base::app | std::ios_base::out);

    std::ofstream log_names(names_path, std::ios_base::app | std::ios_base::out);


    
    
    
    

    
    
    //pcl::io::savePCDFileASCII(save_string +pcd_nr+ "_voxel_passthrough_planes.pcd", *cloud_final);

    if(cloud_proximitate.size()>0){
      //pcl::io::savePCDFileASCII(save_string +pcd_nr+ "_voxel_proximity.pcd", cloud_proximitate);
     
    }
    log_score<<score1<<";"<<score_plane1<<";"<<score_plane2<<";"<<score_plane3<<";"<<'\n';
    log_names<<str<<'\n';
    
    cout << "[*] Conversion finished!" << endl;
    
    
    
 
}

}

  return (0);
}
