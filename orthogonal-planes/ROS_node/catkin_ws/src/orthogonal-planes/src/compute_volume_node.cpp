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
#include <pcl/filters/project_inliers.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <ppfplane/compute_volume_nodeConfig.h>
#include <dynamic_reconfigure/server.h>
#include "std_msgs/String.h"

#define PI 3.14159265

/* -------- DEFS FOR RANSAC PARAMS -------- */


/* ----- END DEFS FOR RANSAC PARAMS ------ */

class ComputeVolumeNode
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ComputeVolumeNode()
  {

    bool ok2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ> all_lines[4][4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

    float Coeficients[3][4];

    float Volum = 1;

    pub1_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_plan", 1);
    pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_proiectii", 1);
    pub3_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_linii", 1);
    pub4_ = nh_.advertise<sensor_msgs::PointCloud2>("/Cutof_Object_points", 1);
    pub5_ = nh_.advertise<sensor_msgs::PointCloud2>("/Planes_wrong_angle", 1);

    sub_ = nh_.subscribe("/pf_out", 1, &ComputeVolumeNode::cloudCallback, this);

    config_server_.setCallback(boost::bind(&ComputeVolumeNode::dynReconfCallback, this, _1, _2));

    vis_pub = nh_.advertise<visualization_msgs::Marker>("/Volum_final", 0);
    vis2_pub = nh_.advertise<visualization_msgs::Marker>("/Nr_of_planes", 0);
    vis3_pub = nh_.advertise<visualization_msgs::Marker>("/1_plane_cases", 0);
    vis4_pub = nh_.advertise<visualization_msgs::Marker>("/camera_type", 0);
    vis5_pub = nh_.advertise<visualization_msgs::Marker>("/Volume_Error", 0);
  }

  ~ComputeVolumeNode() {}


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

  void third_perpendicular_plane(float Coeficients[3][4],
                                 int i,
                                 int j,
                                 float x0,
                                 float y0,
                                 float z0)
  {

    float a1 = Coeficients[i - 1][0];
    float b1 = Coeficients[i - 1][1];
    float c1 = Coeficients[i - 1][2];

    float a2 = Coeficients[j - 1][0];
    float b2 = Coeficients[j - 1][1];
    float c2 = Coeficients[j - 1][2];

    float a3 = b1 * c2 - c1 * b2;
    float b3 = c1 * a2 - a1 * c2;
    float c3 = a1 * b2 - a2 * b1;

    float d3 = a3 * x0 + b3 * y0 + c3 * z0;

    Coeficients[3 - (i - 1) - (j - 1)][0] = a3;
    Coeficients[3 - (i - 1) - (j - 1)][1] = b3;
    Coeficients[3 - (i - 1) - (j - 1)][2] = c3;
    Coeficients[3 - (i - 1) - (j - 1)][3] = -d3;
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

  void compute_length_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           float &distanta,
                           float &coordonate_punct_minim_x,
                           float &coordonate_punct_minim_y,
                           float &coordonate_punct_minim_z,
                           float &coordonate_punct_maxim_x,
                           float &coordonate_punct_maxim_y,
                           float &coordonate_punct_maxim_z)
  {

    float minim_x = cloud->points[0].x;
    int index_min_x = 0;

    float minim_y = cloud->points[0].y;
    int index_min_y = 0;

    float minim_z = cloud->points[0].z;
    int index_min_z = 0;

    float maxim_x = cloud->points[0].x;
    int index_max_x = 0;

    float maxim_y = cloud->points[0].y;
    int index_max_y = 0;

    float maxim_z = cloud->points[0].z;
    int index_max_z = 0;

    for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    {
      if (minim_x > cloud->points[nIndex].x)
      {
        minim_x = cloud->points[nIndex].x;
        index_min_x = nIndex;
      }

      if (minim_y > cloud->points[nIndex].y)
      {
        minim_y = cloud->points[nIndex].y;
        index_min_y = nIndex;
      }

      if (minim_z > cloud->points[nIndex].z)
      {
        minim_z = cloud->points[nIndex].z;
        index_min_z = nIndex;
      }

      if (maxim_x < cloud->points[nIndex].x)
      {
        maxim_x = cloud->points[nIndex].x;
        index_max_x = nIndex;
      }

      if (maxim_y < cloud->points[nIndex].y)
      {
        maxim_y = cloud->points[nIndex].y;
        index_max_y = nIndex;
      }

      if (maxim_z < cloud->points[nIndex].z)
      {
        maxim_z = cloud->points[nIndex].z;
        index_max_z = nIndex;
      }
    }

    float Sortare[3];

    Sortare[0] = abs(maxim_x - minim_x);
    Sortare[1] = abs(maxim_y - minim_y);
    Sortare[2] = abs(maxim_z - minim_z);

    float maximum = Sortare[0];

    float Puncte[2][3];

    int t = 0;

    Puncte[0][0] = index_min_x;
    Puncte[1][0] = index_max_x;
    Puncte[0][1] = index_min_y;
    Puncte[1][1] = index_max_y;
    Puncte[0][2] = index_min_z;
    Puncte[1][2] = index_max_z;

    for (int q = 0; q < 3; q++)
    {
      if (maximum < Sortare[q])
      {
        maximum = Sortare[q];
        t = q;
      }
    }

    int pozitie_min = Puncte[0][t];
    int pozitie_max = Puncte[1][t];

    float distanta_x = (cloud->points[pozitie_max].x - cloud->points[pozitie_min].x);

    distanta_x = distanta_x * distanta_x;

    float distanta_y = (cloud->points[pozitie_max].y - cloud->points[pozitie_min].y);

    distanta_y = distanta_y * distanta_y;

    float distanta_z = (cloud->points[pozitie_max].z - cloud->points[pozitie_min].z);

    distanta_z = distanta_z * distanta_z;

    distanta = distanta_x + distanta_y + distanta_z;

    distanta = sqrt(distanta_x + distanta_y + distanta_z);

    coordonate_punct_minim_x = cloud->points[pozitie_min].x;
    coordonate_punct_minim_y = cloud->points[pozitie_min].y;
    coordonate_punct_minim_z = cloud->points[pozitie_min].z;

    coordonate_punct_maxim_x = cloud->points[pozitie_max].x;
    coordonate_punct_maxim_y = cloud->points[pozitie_max].y;
    coordonate_punct_maxim_z = cloud->points[pozitie_max].z;
  }

  void check_perpendicular(pcl::ModelCoefficients::Ptr plane_1,
                           pcl::ModelCoefficients::Ptr plane_2,
                           double perpendicular_threshold,
                           bool &perp_ok)

  {
    perp_ok = 0;

    float a1 = plane_1->values[0];
    float b1 = plane_1->values[1];
    float c1 = plane_1->values[2];

    float a2 = plane_2->values[0];
    float b2 = plane_2->values[1];
    float c2 = plane_2->values[2];

    float aux = abs(a1 * a2 + b1 * b2 + c1 * c2);

    if (abs(a1 * a2 + b1 * b2 + c1 * c2) < perpendicular_threshold)
    {
      perp_ok = 1;
    }
    else
    {
    }
  }

  void add_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    std::cout << cloud_final->size() << '\n';

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_final);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);

    std::cout << cloud_normals->size() << '\n';

    // cloud_normals->size () should have the same size as the input cloud->size ()*
  }




  void check_passthrough_limits(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
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

        int N = final_pointcloud.width;

        final_pointcloud.width = final_pointcloud.width + 1;
        final_pointcloud.is_dense = false;
        final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

        final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
        final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
        final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
      }

       if (cloud_final->points[nIndex].x > x_upper_limit - threshold_x) 
      {
        nr_puncte[0][1] = nr_puncte[0][1] + 1;

        int N = final_pointcloud.width;

        final_pointcloud.width = final_pointcloud.width + 1;
        final_pointcloud.is_dense = false;
        final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

        final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
        final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
        final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
      }


      if (cloud_final->points[nIndex].y < y_lower_limit + threshold_y)
      {
        nr_puncte[1][0] = nr_puncte[1][0] + 1;

        int N = final_pointcloud.width;

        final_pointcloud.width = final_pointcloud.width + 1;
        final_pointcloud.is_dense = false;
        final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

        final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
        final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
        final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
      }

      if (cloud_final->points[nIndex].y > y_upper_limit - threshold_y)
      {
        nr_puncte[1][1] = nr_puncte[1][1] + 1;

        int N = final_pointcloud.width;

        final_pointcloud.width = final_pointcloud.width + 1;
        final_pointcloud.is_dense = false;
        final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

        final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
        final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
        final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
      }
/*

      if (cloud_final->points[nIndex].z < z_lower_limit + threshold_z) 
      {
        nr_puncte[2][0] = nr_puncte[2][0] + 1;

        int N = final_pointcloud.width;

        final_pointcloud.width = final_pointcloud.width + 1;
        final_pointcloud.is_dense = false;
        final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

        final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
        final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
        final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
      }

     
      
      if (cloud_final->points[nIndex].z > z_upper_limit - threshold_z);
      {
        nr_puncte[2][1] = nr_puncte[2][1] + 1;

        int N = final_pointcloud.width;

        final_pointcloud.width = final_pointcloud.width + 1;
        final_pointcloud.is_dense = false;
        final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

        final_pointcloud.points[N].x = cloud_final->points[nIndex].x;
        final_pointcloud.points[N].y = cloud_final->points[nIndex].y;
        final_pointcloud.points[N].z = cloud_final->points[nIndex].z;
      }
*/
    }

   

  

    
    if (nr_puncte[0][0] >= minimum_nr_points)
    {
      std::cout << "Object cut X min. Move towards X_min" << '\n';
    }

    if (nr_puncte[1][0] >= minimum_nr_points)
    {
      std::cout << "Object cut Y min. Move towards Y_min" << '\n';
    }

    if (nr_puncte[2][0] >= minimum_nr_points)
    {
      std::cout << "Object cut Z min. Move towards Z_min" << '\n';
    }

    if (nr_puncte[0][1] >= minimum_nr_points)
    {
      std::cout << "Object cut X max. Move towards X_max" << '\n';
    }

    if (nr_puncte[1][1] >= minimum_nr_points)
    {
      std::cout << "Object cut Y max. Move towards Y_max" << '\n';
    }

    if (nr_puncte[2][1] >= minimum_nr_points)
    {
      std::cout << "Object cut Z max. Move towards Z_max" << '\n';
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

    if (nr_puncte[0][0] >= minimum_nr_points)
    {
      std::cout << "Object cut X min. Move towards X_min" << '\n';

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

      std::cout<<"Puncte X min:"<<nr_points<<'\n';
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


    if (nr_puncte[0][1] >= minimum_nr_points)
    {
      std::cout << "Object cut X max. Move towards X_max" << '\n';

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

       std::cout<<"Puncte X max:"<<nr_points<<'\n';
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

    if (nr_puncte[1][0] >= minimum_nr_points)
    {
      std::cout << "Object cut Y min. Move towards Y_min" << '\n';

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

       std::cout<<"Puncte Y min:"<<nr_points<<'\n';
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

    if (nr_puncte[1][1] >= minimum_nr_points)
    {
      std::cout << "Object cut Y max. Move towards Y_max" << '\n';

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

       std::cout<<"Puncte Y max:"<<nr_points<<'\n';
    }

  }

  void check_parallel(pcl::ModelCoefficients::Ptr plane_1,
                      pcl::ModelCoefficients::Ptr plane_2,
                      double parallel_threshold,
                      bool &paral_ok)

  {
    paral_ok = 0;

    float a1 = plane_1->values[0];
    float b1 = plane_1->values[1];
    float c1 = plane_1->values[2];

    float a2 = plane_2->values[0];
    float b2 = plane_2->values[1];
    float c2 = plane_2->values[2];

    if ((abs(a1 / a2 - b1 / b2) < parallel_threshold) && (abs(b1 / b2 - c1 / c2) < parallel_threshold) && (abs(a1 / a2 - c1 / c2) < parallel_threshold))
    {
      //std::cout << "Paralel" << '\n';
      paral_ok = 1;
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
          std::cout<<abs(cosine_value)<<" < "<<cos(limit_angle)*(PI)/180<<'\n';

          cos_angle=abs(cosine_value);
      }

      
      

  } 

  void project_plane_2_plane_single(pcl::PointCloud<pcl::PointXYZ>::Ptr plane,
                                    float Coeficients[3][4],
                                    int j,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected)
  {

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);

    coefficients->values[0] = Coeficients[j - 1][0];
    coefficients->values[1] = Coeficients[j - 1][1];
    coefficients->values[2] = Coeficients[j - 1][2];
    coefficients->values[3] = Coeficients[j - 1][3];

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(plane);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
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

  void create_lines(float Coeficients[3][4],
                    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],
                    pcl::PointCloud<pcl::PointXYZ> all_lines[4][4],
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii,
                    bool &ok2,
                    bool &ok_lines)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string str;
    std::string str2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    int i, j;

    if (ok2 != 0)
    {

      for (i = 1; i < 4; i++)
      {

        cloud = all_planes[i];

        // Create a set of planar coefficients with X=Y=0,Z=1
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);

        for (j = 1; j < 4; j++)
        {
          if (j != i)
          {
            /*
        std::cout << "\n";
        std::cout << "plan " << i << "\n";
         */
            coefficients->values[0] = Coeficients[j - 1][0];
            coefficients->values[1] = Coeficients[j - 1][1];
            coefficients->values[2] = Coeficients[j - 1][2];
            coefficients->values[3] = Coeficients[j - 1][3];

            /*  
        std::cout << "Projecting plane " << i << " to plane " << j << "\n";
        std::cout << "Saving line " << i << "_" << j << "\n";
         */
            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(cloud);
            proj.setModelCoefficients(coefficients);
            proj.filter(*cloud_projected);

            //PCL_INFO("Saving the projected Pointcloud \n");

            all_lines[i][j] = *cloud_projected;

            *cloud_linii = *cloud_linii + *cloud_projected;

            if (all_lines[i][j].width < 10)
            {
              ok_lines = 0;
            }
          }
        }
      }
    }
    else
    {
      std::cout << "Cannot segment"
                << "\n";
    }
  }

  void project_line_2_plane(float Coeficients[3][4],
                            pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],
                            pcl::PointCloud<pcl::PointXYZ> all_lines[4][4],
                            pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4],
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii,
                            bool &ok_lines)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int i, j;

    int aux;

    for (i = 1; i < 3; i++)
    {
      for (j = i; j < 4; j++)
      {
      }
    }

    for (i = 1; i < 3; i++)
    {
      for (j = i; j < 4; j++)
      {

        if (i != j)
        {

          aux = (6 - i - j);

          // Create a set of planar coefficients with X=Y=0,Z=1
          pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients());
          coefficients2->values.resize(4);

          coefficients2->values[0] = Coeficients[aux - 1][0];
          coefficients2->values[1] = Coeficients[aux - 1][1];
          coefficients2->values[2] = Coeficients[aux - 1][2];
          coefficients2->values[3] = Coeficients[aux - 1][3];

          int aux2;
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZ>);

          for (int z = 1; z < 3; z++)
          {

            aux2 = i;
            i = j;
            j = aux2;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

            *cloud = all_lines[i][j];

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(cloud);
            proj.setModelCoefficients(coefficients2);
            proj.filter(*cloud_projected);

            *cloud_c += *cloud;
            *cloud_c += *cloud_projected;

            *cloud_proiectii += *cloud_c;
          }

          all_projected_lines[i][j] = cloud_c;

          if (all_projected_lines[i][j]->width < 10)
          {
            ok_lines = 0;
          }
        }
      }
    }
  }

  void compute_volume(pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4],
                      float &Volum)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    float muchii[3];

    int i, j;

    for (i = 1; i < 3; i++)
    {
      for (j = 1; j < 4; j++)
      {

        if (i < j)
        {

          cloud = all_projected_lines[i][j];

          float coordonate_punct_minim_x;
          float coordonate_punct_minim_y;
          float coordonate_punct_minim_z;
          float coordonate_punct_maxim_x;
          float coordonate_punct_maxim_y;
          float coordonate_punct_maxim_z;

          float distanta;

          compute_length_line(cloud, distanta,
                              coordonate_punct_minim_x,
                              coordonate_punct_minim_y,
                              coordonate_punct_minim_z,
                              coordonate_punct_maxim_x,
                              coordonate_punct_maxim_y,
                              coordonate_punct_maxim_z);

          Volum = Volum * distanta;
        }
      }
    }

    std::cout << "Volum final " << Volum << " m^3"
              << "\n";
  }

  void compute_volume_1_plane(pcl::ModelCoefficients::Ptr coefficients_floor,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                              float &Volum)
  {
    float point_x = 0;
    float point_y = 0;
    float point_z = 0;

    int nIndex;

    for (nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {
      point_x = point_x + cloud_final->points[nIndex].x;
      point_y = point_y + cloud_final->points[nIndex].y;
      point_z = point_z + cloud_final->points[nIndex].z;
    }

    point_x = point_x / nIndex;
    point_y = point_y / nIndex;
    point_z = point_z / nIndex;

    float distanta;

    distanta = abs(-(coefficients_floor->values[0] * point_x) - (coefficients_floor->values[1] * point_y) - (coefficients_floor->values[2] * point_z));

    //std::cout<<"Coordonate centru"<<point_x<<" "<<point_y<<" "<<point_z<<'\n';

    float distanta_maxima_1 = -5000;

    float point_x1;
    float point_y1;
    float point_z1;

    for (nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {
      float dist_x = point_x - cloud_final->points[nIndex].x;
      float dist_y = point_y - cloud_final->points[nIndex].y;
      float dist_z = point_z - cloud_final->points[nIndex].z;

      float dist_aux = sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
      if (dist_aux > distanta_maxima_1)
      {
        distanta_maxima_1 = dist_aux;

        point_x1 = cloud_final->points[nIndex].x;
        point_y1 = cloud_final->points[nIndex].y;
        point_z1 = cloud_final->points[nIndex].z;
      }
    }

    //std::cout<<"Coordonate punct 1 "<<point_x1<<" "<<point_y1<<" "<<point_z1<<'\n';

    float distanta_maxima_2 = -5000;

    float point_x2;
    float point_y2;
    float point_z2;

    for (nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {
      float dist_x2 = point_x1 - cloud_final->points[nIndex].x;
      float dist_y2 = point_y1 - cloud_final->points[nIndex].y;
      float dist_z2 = point_z1 - cloud_final->points[nIndex].z;

      float dist_aux_2 = sqrt(dist_x2 * dist_x2 + dist_y2 * dist_y2 + dist_z2 * dist_z2);
      if (dist_aux_2 > distanta_maxima_2)
      {
        distanta_maxima_2 = dist_aux_2;

        point_x2 = cloud_final->points[nIndex].x;
        point_y2 = cloud_final->points[nIndex].y;
        point_z2 = cloud_final->points[nIndex].z;
      }
    }

    //std::cout<<"Coordonate punct 2 "<<point_x2<<" "<<point_y2<<" "<<point_z2<<'\n';

    float dist_hypo_x = point_x2 - point_x1;
    float dist_hypo_y = point_y2 - point_y1;
    float dist_hypo_z = point_z2 - point_z1;

    float hypot = dist_hypo_x * dist_hypo_x + dist_hypo_y * dist_hypo_y + dist_hypo_z * dist_hypo_z;

    float suma_minima = 5000;

    float cateta_1_x;
    float cateta_1_y;
    float cateta_1_z;
    float cateta_2_x;
    float cateta_2_y;
    float cateta_2_z;

    float cateta_1;
    float cateta_2;

    float minimizare;

    float point_x3;
    float point_y3;
    float point_z3;

    float cateta_1_final;
    float cateta_2_final;

    for (nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
    {

      if (((cloud_final->points[nIndex].x != point_x1) || (cloud_final->points[nIndex].y != point_y1) || (cloud_final->points[nIndex].z != point_z1)) &&
          ((cloud_final->points[nIndex].x != point_x2) || (cloud_final->points[nIndex].y != point_y2) || (cloud_final->points[nIndex].z != point_z2))) //Daca punctele nu sunt egale cu capetele
      {
        cateta_1_x = point_x1 - cloud_final->points[nIndex].x;
        cateta_1_y = point_y1 - cloud_final->points[nIndex].y;
        cateta_1_z = point_z1 - cloud_final->points[nIndex].z;

        cateta_1 = (cateta_1_x * cateta_1_x + cateta_1_y * cateta_1_y + cateta_1_z * cateta_1_z);

        cateta_2_x = point_x2 - cloud_final->points[nIndex].x;
        cateta_2_y = point_y2 - cloud_final->points[nIndex].y;
        cateta_2_z = point_z2 - cloud_final->points[nIndex].z;

        cateta_2 = (cateta_2_x * cateta_2_x + cateta_2_y * cateta_2_y + cateta_2_z * cateta_2_z);

        minimizare = cateta_1 + cateta_2 - hypot;

        if (abs(minimizare) < suma_minima)
        {
          suma_minima = minimizare;

          point_x3 = cloud_final->points[nIndex].x;
          point_y3 = cloud_final->points[nIndex].y;
          point_z3 = cloud_final->points[nIndex].z;

          cateta_1_final = cateta_1;
          cateta_2_final = cateta_2;
        }
      }
    }

    //std::cout<<"Coordonate punct 3 "<<point_x3<<" "<<point_y3<<" "<<point_z3<<'\n';

    Volum = cateta_1_final * cateta_2_final * distanta;

    // std::cout<<"Cateta 1 "<<cateta_1_final<<'\n';
    //std::cout<<"Cateta 2 "<<cateta_2_final<<'\n';
    //std::cout<<"Ipotenuza:"<<hypot<<'\n';
    // std::cout<<"Minimizare:"<<minimizare<<'\n';
    // std::cout<<"Inaltime:"<<distanta<<'\n';
    std::cout << "Volum" << Volum << '\n';
    //std::cout<<'\n';
  }

  void compute_volume_2_planes(float Coeficients[3][4],
                               int i,
                               int j,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],
                               pcl::PointCloud<pcl::PointXYZ> all_lines[4][4],
                               pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4],
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii,
                               bool &ok_lines,
                               float &Volum)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr projection(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);

    plane = all_planes[i];

    project_plane_2_plane_single(plane, Coeficients, j, projection);

    *line += *projection;

    *cloud_linii += *projection;

    all_lines[i][j] = *projection;

    plane = all_planes[j];

    project_plane_2_plane_single(plane, Coeficients, i, projection);

    *line += *projection;

    *cloud_linii += *projection;

    all_lines[j][i] = *projection;

    float coordonate_punct_minim_x;
    float coordonate_punct_minim_y;
    float coordonate_punct_minim_z;
    float coordonate_punct_maxim_x;
    float coordonate_punct_maxim_y;
    float coordonate_punct_maxim_z;

    float distanta;

    if (line->width > 10)
    {
      compute_length_line(line,
                          distanta,
                          coordonate_punct_minim_x,
                          coordonate_punct_minim_y,
                          coordonate_punct_minim_z,
                          coordonate_punct_maxim_x,
                          coordonate_punct_maxim_y,
                          coordonate_punct_maxim_z);

      third_perpendicular_plane(Coeficients, i, j, coordonate_punct_maxim_x, coordonate_punct_maxim_y, coordonate_punct_maxim_z);

      plane = all_planes[1];

      int coef_remaining = 6 - i - j;

      project_plane_2_plane_single(plane, Coeficients, coef_remaining, projection);

      all_lines[1][coef_remaining] = *projection;
      all_lines[coef_remaining][1] = *projection;

      *cloud_linii += *projection;

      plane = all_planes[j];

      project_plane_2_plane_single(plane, Coeficients, coef_remaining, projection);

      all_lines[2][coef_remaining] = *projection;
      all_lines[coef_remaining][2] = *projection;

      *cloud_linii += *projection;

      project_line_2_plane(Coeficients, all_planes, all_lines, all_projected_lines, cloud_proiectii, ok_lines);

      if (ok_lines)
      {
        compute_volume(all_projected_lines, Volum);
      }
    }
    else
    {
      std::cout << "Cannot create line" << '\n';
    }
  }

 float score_passthrough_side(int p,pcl::PointCloud<pcl::PointXYZ> cloud_proximitate, int limit_number)
    {
      if (p>=2)
      {
          if (cloud_proximitate.size()>limit_number)
           {
             return(1 / cloud_proximitate.size()) ;
           }
          else
          {
            return(1);
          }
      }
      else
      {
        return(0);
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
            return(1);
          }
      }
      else
      {
        return(0);
      }
    }
 /*
  float score_distance(int p,int plane_nr, float distance ,pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4])
  {
     if (p>plane_nr)
     {

     }
  }

  */
 
  void compute_all(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii,
                   float &Volum,
                   int &p,
                   bool &perp_ok,
                   bool &paral_ok,
                   pcl::PointCloud<pcl::PointXYZ> &cloud_proximitate,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_wrong_angle)
  {

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

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    float Coeficients[3][4];

    pcl::PointCloud<pcl::PointXYZ> all_lines[4][4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];

    pcl::ModelCoefficients::Ptr coefficients_floor(new pcl::ModelCoefficients);

    Eigen::Vector3f normal_floor;

    int x_min_nr_points=0;
    int x_max_nr_points=0;
    int y_min_nr_points=0;
    int y_max_nr_points=0;

    float cos_angle_u1=1;
    float cos_angle_u2=1;
    float cos_angle_u3=1;



    bool ok = 1;

    bool ok2 = 1;

    bool ok_lines = 1;

    pcl::PCDWriter writer;

    if (cloud->size() < nivel_initial)
    {
      ok = 0;
      p = 0; // NO PLANE
    }
    else
    {
      planar_segmenting_single_time(cloud, cloud_floor, coefficients_floor);

      normal_floor << coefficients_floor->values[0], coefficients_floor->values[1], coefficients_floor->values[2];
      if (coefficients_floor->values[3] < 0)
      {
        normal_floor *= -1;
      }
      p = 1;
    }

    for (int t = 1; (t < 4) && ok; t++)
    {

      euclidean_segmenting(cloud, cloud_f, ok2);

      if (cloud_f->size() < nivel_initial / dividing_number)
      {
        ok = 0; //No more planes to cut out
      }

      if (ok2 != 0)
      {
        planar_segmenting(cloud_f, Coeficients, all_planes, cloud_final, t, ok2);

        cloud = cloud_f; // Cloud is now the extracted pointcloud

        if (cloud->size() < nivel_initial / dividing_number)
        {
          ok = 0;
          if (t == 1)
          {
            p = 1; // Only the Ground Plane
          }
        }

        p = t + 1;
      }
    }
    // if (p >= 2)
    // {
    //   check_passthrough_limits_x_min(cloud_final,
    //                            threshold_x,
    //                            threshold_y,
    //                            threshold_z,
    //                            z_lower_limit,
    //                            z_upper_limit,
    //                            y_lower_limit,
    //                            y_upper_limit,
    //                            x_lower_limit,
    //                            x_upper_limit,
    //                            minimum_nr_points,
    //                            x_min_nr_points,
    //                            cloud_proximitate_x_min);

    //   cloud_proximitate=cloud_proximitate+cloud_proximitate_x_min;

    //   check_passthrough_limits_x_max(cloud_final,
    //                            threshold_x,
    //                            threshold_y,
    //                            threshold_z,
    //                            z_lower_limit,
    //                            z_upper_limit,
    //                            y_lower_limit,
    //                            y_upper_limit,
    //                            x_lower_limit,
    //                            x_upper_limit,
    //                            minimum_nr_points,
    //                            x_max_nr_points,
    //                            cloud_proximitate_x_max);

    //   cloud_proximitate+=cloud_proximitate_x_max;

    //   check_passthrough_limits_y_min(cloud_final,
    //                            threshold_x,
    //                            threshold_y,
    //                            threshold_z,
    //                            z_lower_limit,
    //                            z_upper_limit,
    //                            y_lower_limit,
    //                            y_upper_limit,
    //                            x_lower_limit,
    //                            x_upper_limit,
    //                            minimum_nr_points,
    //                            y_min_nr_points,
    //                            cloud_proximitate_y_min);

    //   cloud_proximitate+=cloud_proximitate_y_min;

    //   check_passthrough_limits_y_max(cloud_final,
    //                            threshold_x,
    //                            threshold_y,
    //                            threshold_z,
    //                            z_lower_limit,
    //                            z_upper_limit,
    //                            y_lower_limit,
    //                            y_upper_limit,
    //                            x_lower_limit,
    //                            x_upper_limit,
    //                            minimum_nr_points,
    //                            y_max_nr_points,
    //                            cloud_proximitate_y_max);

    //   cloud_proximitate+=cloud_proximitate_y_max;

    // }

    if (p == 2)
    {  /*
        compute_angle(all_planes,
                   Coeficients,
                   1,
                  angle,
                  cloud_wrong_angle);

          */


      pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
      coefficients_plane->values.resize(4);

      coefficients_plane->values[0] = Coeficients[0][0];
      coefficients_plane->values[1] = Coeficients[0][1];
      coefficients_plane->values[2] = Coeficients[0][2];
      coefficients_plane->values[3] = Coeficients[0][3];

      check_parallel(coefficients_floor,
                     coefficients_plane,
                     parallel_threshold,
                     paral_ok);

      check_perpendicular(coefficients_floor,
                          coefficients_plane,
                          parallel_threshold,
                          perp_ok);


    }

    if (p == 3)
    {

    compute_angle(all_planes,
                   Coeficients,
                   1,
                  angle,
                  cos_angle_u1,
                  cloud_wrong_angle);

    compute_angle(all_planes,
                   Coeficients,
                   2,
                  angle,
                  cos_angle_u2,
                  cloud_wrong_angle);



      // std::cout << "2 Planuri" << '\n';

      bool is_perp = 0;

      pcl::ModelCoefficients::Ptr plane_1(new pcl::ModelCoefficients);
      plane_1->values.resize(4);

      plane_1->values[0] = Coeficients[0][0];
      plane_1->values[1] = Coeficients[0][1];
      plane_1->values[2] = Coeficients[0][2];
      plane_1->values[3] = Coeficients[0][3];

      pcl::ModelCoefficients::Ptr plane_2(new pcl::ModelCoefficients);
      plane_2->values.resize(4);

      plane_2->values[0] = Coeficients[1][0];
      plane_2->values[1] = Coeficients[1][1];
      plane_2->values[2] = Coeficients[1][2];
      plane_2->values[3] = Coeficients[1][3];

      check_perpendicular(plane_1, plane_2, perpendicular_threshold, is_perp);

      if (is_perp)
      {
        compute_volume_2_planes(Coeficients,
                                1,
                                2,
                                all_planes,
                                all_lines,
                                all_projected_lines,
                                cloud_proiectii,
                                cloud_linii,
                                ok_lines,
                                Volum);
      }

      bool paral_floor_1 = 0;
      bool paral_floor_2 = 0;

      check_parallel(coefficients_floor, plane_1, perpendicular_threshold, paral_floor_1);
      check_parallel(coefficients_floor, plane_2, perpendicular_threshold, paral_floor_2);
      /*
      if (paral_floor_1)
      {
        compute_volume_1_plane(coefficients_floor, all_planes[1], Volum);
        std::cout << "2 Planuri neperpendiculare, 1 plan paralel cu podeaua" << '\n';
      }

      if (paral_floor_2)
      {
        compute_volume_1_plane(coefficients_floor, all_planes[2], Volum);
        std::cout << "2 Planuri neperpendiculare, 1 plan paralel cu podeaua" << '\n';
      }
      */
      //add_normals(cloud_final,cloud_normals);
    }

    if (p == 4)
    {
      
      compute_angle(all_planes,
                   Coeficients,
                   1,
                  angle,
                  cos_angle_u1,
                  cloud_wrong_angle);

    compute_angle(all_planes,
                   Coeficients,
                   2,
                  angle,
                  cos_angle_u2,
                  cloud_wrong_angle);
      /*
      compute_angle(all_planes,
                   Coeficients,
                   3,
                  angle,
                  cos_angle_u3,
                  cloud_wrong_angle);

        */
      

      bool is_perp_12 = 0;
      bool is_perp_13 = 0;
      bool is_perp_23 = 0;

      pcl::ModelCoefficients::Ptr plane_1(new pcl::ModelCoefficients);
      plane_1->values.resize(4);

      plane_1->values[0] = Coeficients[0][0];
      plane_1->values[1] = Coeficients[0][1];
      plane_1->values[2] = Coeficients[0][2];
      plane_1->values[3] = Coeficients[0][3];

      pcl::ModelCoefficients::Ptr plane_2(new pcl::ModelCoefficients);
      plane_2->values.resize(4);

      plane_2->values[0] = Coeficients[1][0];
      plane_2->values[1] = Coeficients[1][1];
      plane_2->values[2] = Coeficients[1][2];
      plane_2->values[3] = Coeficients[1][3];

      pcl::ModelCoefficients::Ptr plane_3(new pcl::ModelCoefficients);
      plane_3->values.resize(4);

      plane_3->values[0] = Coeficients[2][0];
      plane_3->values[1] = Coeficients[2][1];
      plane_3->values[2] = Coeficients[2][2];
      plane_3->values[3] = Coeficients[2][3];

      check_perpendicular(plane_1, plane_2, perpendicular_threshold, is_perp_12);
      check_perpendicular(plane_1, plane_3, perpendicular_threshold, is_perp_13);
      check_perpendicular(plane_2, plane_3, perpendicular_threshold, is_perp_23);

      if (is_perp_12 && is_perp_13 && is_perp_23)
      {

       
        compute_volume_2_planes(Coeficients,
                                1,
                                2,
                                all_planes,
                                all_lines,
                                all_projected_lines,
                                cloud_proiectii,
                                cloud_linii,
                                ok_lines,
                                Volum);
      }
      else
      {
        if (is_perp_12)
        {
          compute_volume_2_planes(Coeficients,
                                  1,
                                  2,
                                  all_planes,
                                  all_lines,
                                  all_projected_lines,
                                  cloud_proiectii,
                                  cloud_linii,
                                  ok_lines,
                                  Volum);
        }
        else
        {
          if (is_perp_13)
          {
            compute_volume_2_planes(Coeficients,
                                    1,
                                    3,
                                    all_planes,
                                    all_lines,
                                    all_projected_lines,
                                    cloud_proiectii,
                                    cloud_linii,
                                    ok_lines,
                                    Volum);
          }
          else
          {
            if (is_perp_23)
            {
              compute_volume_2_planes(Coeficients,
                                      2,
                                      3,
                                      all_planes,
                                      all_lines,
                                      all_projected_lines,
                                      cloud_proiectii,
                                      cloud_linii,
                                      ok_lines,
                                      Volum);
            }
          }
        }
      }
    }

    float score1;
    float score2;
    float score3;
    float score4;

    // score1=score_passthrough_side(p,cloud_proximitate_x_min,minimum_nr_points)+
    //       score_passthrough_side(p,cloud_proximitate_x_max,minimum_nr_points)+
    //       score_passthrough_side(p,cloud_proximitate_y_min,minimum_nr_points)+
    //       score_passthrough_side(p,cloud_proximitate_y_max,minimum_nr_points);
    // std::cout<<"Score 1:"<<score1<<'\n';

    // score2=score_angle(p,1,cos_angle_u1,angle)+
    //        score_angle(p,2,cos_angle_u2,angle);

    // std::cout<<"Score 2:"<<score2<<'\n';
  }

  void
  dynReconfCallback(ppfplane::compute_volume_nodeConfig &config, uint32_t level)
  {
    selection_camera = config.selection_camera;

    nivel_initial = config.nr_points_initial;
    dividing_number = config.dividing_number;
    perpendicular_threshold = config.perpendicular_threshold;
    parallel_threshold = config.parallel_threshold;

    threshold_x = config.threshold_x;
    threshold_y = config.threshold_y;
    threshold_z = config.threshold_z;
    minimum_nr_points = (int)config.minimum_nr_points;

    z_lower_limit = config.z_lower_limit; 
    z_upper_limit = config.z_upper_limit;
    x_lower_limit = config.x_lower_limit;
    x_upper_limit = config.x_upper_limit;
    y_lower_limit = config.y_lower_limit;
    y_upper_limit = config.y_upper_limit;

    angle=config.angle_threshold;

    real_volume=config.real_volume;

    
    /*
    if (nh_.hasParam("/Dimensions"))
      {
   std::cout<<"Exista";
      }
      */
   
  }

  void
  set_marker(visualization_msgs::Marker &marker)
  {
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = 1;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha! Otherwise it is invisible
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
  }
  /*
  void cloudCallback2(const std_msgs::String::ConstPtr& msg){

      std::stringstream ss(msg->data.c_str());
      for(int i=0;i<5;i++){
           ss >> passthrough_limits[i];
          }

  }
  */

  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {

    float Volum = 1;
    int p = 0;

    bool paral_ok = 0;
    bool perp_ok = 0;
    ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_Test;
    pcl::fromROSMsg(*cloud_msg, cloud_Test);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plan_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plan_2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPTR = cloud_Test;

    pcl::PointCloud<pcl::PointXYZ> cloud_passthrough_thresshold;

    cloud_passthrough_thresshold.width = 0;
    cloud_passthrough_thresshold.height = 1;
    cloud_passthrough_thresshold.is_dense = false;
    cloud_passthrough_thresshold.resize(cloud_passthrough_thresshold.width * cloud_passthrough_thresshold.height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_wrong_angle(new pcl::PointCloud<pcl::PointXYZ>);

    compute_all(cloudPTR, cloud_floor, cloud_final, cloud_proiectii, cloud_linii, Volum, p, perp_ok, paral_ok, cloud_passthrough_thresshold,cloud_wrong_angle);

    sensor_msgs::PointCloud2 tempROSMsg;
    sensor_msgs::PointCloud2 tempROSMsg2;
    sensor_msgs::PointCloud2 tempROSMsg3;
    sensor_msgs::PointCloud2 tempROSMsg4;
    sensor_msgs::PointCloud2 tempROSMsg5;

    pcl::toROSMsg(*cloud_final, tempROSMsg);
    pcl::toROSMsg(*cloud_proiectii, tempROSMsg2);
    pcl::toROSMsg(*cloud_linii, tempROSMsg3);
    pcl::toROSMsg(cloud_passthrough_thresshold, tempROSMsg4);
    pcl::toROSMsg(*cloud_wrong_angle, tempROSMsg5);

    //Camera_type
    ////////////////////////////////////////
    std::stringstream ss_camera;
    std::stringstream header_camera;

    if (selection_camera < 1.5) 
    {
      ss_camera << "Openni";
      //header_camera << "camera_depth_optical_frame";
      header_camera << "base_link";
    }
    else
    {
      ss_camera << "Pico";
      header_camera << "pico_zense_depth_frame";
    }

    visualization_msgs::Marker marker_camera;
    marker_camera.header.frame_id = header_camera.str();
    marker_camera.text = ss_camera.str();
    set_marker(marker_camera);

    tempROSMsg.header.frame_id = header_camera.str();
    tempROSMsg2.header.frame_id = header_camera.str();
    tempROSMsg3.header.frame_id = header_camera.str();
    tempROSMsg4.header.frame_id = header_camera.str();
    tempROSMsg5.header.frame_id = header_camera.str();

    //Message Marker Volume
    ////////////////////////////////////////
    std::stringstream ss;

    ss << "Volumul este " << Volum << " m3";

    visualization_msgs::Marker marker;
    marker.header.frame_id = header_camera.str();
    marker.text = ss.str();
    set_marker(marker);

    //Message Marker Volume
    ////////////////////////////////////////
    std::stringstream ss2;

    switch (p)
    {
    case 0:
      ss2 << "No plane detected"
          << "\n";
      break;
    case 1:
      ss2 << "Only ground plane detected"
          << "\n";
      break;
    case 2:
      ss2 << "Ground plane and 1 plane detected"
          << "\n";
      break;
    case 3:
      ss2 << "Ground plane and 2 planes detected"
          << "\n";
      break;
    case 4:
      ss2 << "Ground plane and 3 planes detected"
          << "\n";
      break;
    }

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = header_camera.str();
    marker2.text = ss2.str();
    set_marker(marker2);

    //////////////////////
    std::stringstream ss3;

    ss3 << "Perpendicular " << perp_ok << " si Paralel " << paral_ok;


    visualization_msgs::Marker marker3;
    marker3.header.frame_id = header_camera.str();
    marker3.text = ss3.str();
    set_marker(marker3);

    
    
    std::stringstream ss4;
     ss4 << "Error percentage volume: " << (real_volume-Volum)/real_volume *100<< "%";


    visualization_msgs::Marker marker4;
    marker4.header.frame_id = header_camera.str();
    marker4.text = ss4.str();
    set_marker(marker4);

    

    ///////////////

    //Publish the data

    pub1_.publish(tempROSMsg);
    pub2_.publish(tempROSMsg2);
    pub3_.publish(tempROSMsg3);
    pub4_.publish(tempROSMsg4);
    pub5_.publish(tempROSMsg5);

    vis_pub.publish(marker);
    vis2_pub.publish(marker2);
    vis3_pub.publish(marker3);
    vis4_pub.publish(marker_camera);
    vis5_pub.publish(marker4);

    cloud_final->clear();
    cloud_linii->clear();
    cloud_proiectii->clear();

    Volum = 1;
  }

private:
  bool ok2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii;

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers;
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;

  pcl::PointCloud<pcl::PointXYZ> all_lines[4][4];
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

  float Coeficients[3][4];

  float Volum;

  bool paral_ok;
  bool perp_ok;

  dynamic_reconfigure::Server<ppfplane::compute_volume_nodeConfig> config_server_;

  double nivel_initial;
  double dividing_number;
  double perpendicular_threshold;
  double parallel_threshold;

  double threshold_x;
  double threshold_y;
  double threshold_z;
  int minimum_nr_points;

  float passthrough_limits[6];

  float z_lower_limit;
  float z_upper_limit;
  float y_lower_limit;
  float y_upper_limit;
  float x_lower_limit;
  float x_upper_limit;

   float angle;

  float real_volume;


  std::string dimensions;

  double selection_camera;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Subscriber sub_floats;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub3_;
  ros::Publisher pub4_;
  ros::Publisher pub5_;

  ros::Publisher vis_pub;
  ros::Publisher vis2_pub;
  ros::Publisher vis3_pub;
  ros::Publisher vis4_pub;
  ros::Publisher vis5_pub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_volume_node");

  ComputeVolumeNode vf;

  ros::spin();
}
