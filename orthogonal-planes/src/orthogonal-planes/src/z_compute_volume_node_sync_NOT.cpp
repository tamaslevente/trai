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

#include <pcl_tutorial/compute_volume_nodeConfig.h>
#include <dynamic_reconfigure/server.h>
 #include "std_msgs/String.h"

 #include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>


using namespace message_filters;

 typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, std_msgs::String> MySyncPolicy;


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

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_(nh_, "/pf_out", 1000);
    message_filters::Subscriber<std_msgs::String> sub_float(nh_, "/Dimensions", 1000);


     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_, sub_float);

     sync.registerCallback(boost::bind(&ComputeVolumeNode::cloudCallback, _1, _2));

    //sub_ = nh_.subscribe("/pf_out", 1, &ComputeVolumeNode::cloudCallback, this);
    

    config_server_.setCallback(boost::bind(&ComputeVolumeNode::dynReconfCallback, this, _1, _2));

    vis_pub = nh_.advertise<visualization_msgs::Marker>("/Volum_final", 0);
    vis2_pub = nh_.advertise<visualization_msgs::Marker>("/Nr_of_planes", 0);
    vis3_pub = nh_.advertise<visualization_msgs::Marker>("/1_plane_cases", 0);
    vis4_pub = nh_.advertise<visualization_msgs::Marker>("/camera_type", 0);
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
      //std::cout << "Perpendicular" << '\n';
      perp_ok = 1;
    }
    else
    {
     // std::cout << "Nu e Perpendicular" << '\n';
    }

    std::cout << aux << '\n';
  }

  void add_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
   std::cout<<cloud_final->size()<<'\n';
    

     // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setInputCloud (cloud_final);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  std::cout<<cloud_normals->size()<<'\n';

  // cloud_normals->size () should have the same size as the input cloud->size ()*

  }




  void check_passthrough_limits(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                                float threshold_x,
                                float threshold_y,
                                float threshold_z,
                                int minimum_nr_points)
  {
    int nr_puncte[3][2];

    for(int i=0;i<3;i++){
      for(int j=0;j<2;j++)
      {
        nr_puncte[i][j]=0;
      }
    }

      for (int nIndex = 0; nIndex < cloud_final->points.size(); nIndex++)
      {
        
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
      std::cout << "Paralel" << '\n';
      paral_ok = 1;
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

  void compute_volume_2_planes(float Coeficients[3][4],
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

    plane = all_planes[1];

    project_plane_2_plane_single(plane, Coeficients, 2, projection);

    *line += *projection;

    *cloud_linii += *projection;

    all_lines[1][2] = *projection;

    plane = all_planes[2];

    project_plane_2_plane_single(plane, Coeficients, 1, projection);

    *line += *projection;

    *cloud_linii += *projection;

    all_lines[2][1] = *projection;

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

      third_perpendicular_plane(Coeficients, 1, 2, coordonate_punct_maxim_x, coordonate_punct_maxim_y, coordonate_punct_maxim_z);

      plane = all_planes[1];

      project_plane_2_plane_single(plane, Coeficients, 3, projection);

      all_lines[1][3] = *projection;
      all_lines[3][1] = *projection;

      *cloud_linii += *projection;

      plane = all_planes[2];

      project_plane_2_plane_single(plane, Coeficients, 3, projection);

      all_lines[2][3] = *projection;
      all_lines[3][2] = *projection;

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

  void compute_all(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii,
                   float &Volum, int &p, bool &perp_ok, bool &paral_ok)
  {

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    float Coeficients[3][4];

    pcl::PointCloud<pcl::PointXYZ> all_lines[4][4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];

    pcl::ModelCoefficients::Ptr coefficients_floor(new pcl::ModelCoefficients);

    Eigen::Vector3f normal_floor;

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

    for (int t = 1; (t < 3) && ok; t++)
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

    if (p == 2)
    {

      
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

      std::cout << "2 Planuri" << '\n';

      bool is_perp = 0;
      /* 
        pcl::ModelCoefficients::Ptr plane_1(new pcl::ModelCoefficients);
        plane_1->values.resize(4);

        plane_1->values[0]=Coeficients[0][0];
        plane_1->values[1]=Coeficients[0][1];
        plane_1->values[2]=Coeficients[0][2];
        plane_1->values[3]=Coeficients[0][3];
      
        pcl::ModelCoefficients::Ptr plane_2(new pcl::ModelCoefficients);
        plane_2->values.resize(4);

        plane_2->values[0]=Coeficients[0][0];
        plane_2->values[1]=Coeficients[0][1];
        plane_2->values[2]=Coeficients[0][2];
        plane_2->values[3]=Coeficients[0][3];
       


      
     
        check_perpendicular(plane_1,plane_2,perpendicular_threshold,is_perp);

        */
      /*
        
        if(is_perp)
        {
            compute_volume_2_planes(Coeficients,
                            all_planes,
                            all_lines,
                            all_projected_lines,
                            cloud_proiectii,
                            cloud_linii,
                            ok_lines,
                            Volum);
        }
        */

      compute_volume_2_planes(Coeficients,
                              all_planes,
                              all_lines,
                              all_projected_lines,
                              cloud_proiectii,
                              cloud_linii,
                              ok_lines,
                              Volum);

    add_normals(cloud_final,cloud_normals);

      
    }

    if (p == 4)
    {
      std::cout << "3 Planuri" << '\n';

      create_lines(Coeficients, all_planes, all_lines, cloud_linii, ok2, ok_lines);

      /*compute_volume_2_planes(Coeficients,
                            all_planes,
                            all_lines,
                            all_projected_lines,
                            cloud_proiectii,
                            cloud_linii,
                            ok_lines,
                            Volum);*/

      project_line_2_plane(Coeficients, all_planes, all_lines, all_projected_lines, cloud_proiectii, ok_lines);

      if (ok_lines)
      {
        compute_volume(all_projected_lines, Volum);
      }
    }
  }

  void
  dynReconfCallback(pcl_tutorial::compute_volume_nodeConfig &config, uint32_t level)
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

     z_lower_limit=config.z_lower_limit;
     z_upper_limit=config.z_upper_limit;
     x_lower_limit=config.x_lower_limit;
     x_upper_limit=config.x_upper_limit;
     y_lower_limit=config.y_lower_limit;
     y_upper_limit=config.y_upper_limit;

     passthrough_limits[0]=config.x_lower_limit;
     passthrough_limits[1]=config.x_upper_limit;
     passthrough_limits[2]=config.y_lower_limit;
     passthrough_limits[3]=config.y_upper_limit;
     passthrough_limits[4]=config.z_lower_limit;
     passthrough_limits[5]=config.z_upper_limit;
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



  void
  cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, const std_msgs::StringPtr &msg)
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

    compute_all(cloudPTR, cloud_floor, cloud_final, cloud_proiectii, cloud_linii, Volum, p, perp_ok, paral_ok);

    sensor_msgs::PointCloud2 tempROSMsg;
    sensor_msgs::PointCloud2 tempROSMsg2;
    sensor_msgs::PointCloud2 tempROSMsg3;

    pcl::toROSMsg(*cloud_final, tempROSMsg);
    pcl::toROSMsg(*cloud_proiectii, tempROSMsg2);
    pcl::toROSMsg(*cloud_linii, tempROSMsg3);

    //Camera_type
    ////////////////////////////////////////
    std::stringstream ss_camera;
    std::stringstream header_camera;

    if (selection_camera < 1.5)
    {
      ss_camera << "Openni";
      header_camera << "camera_depth_optical_frame";
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



      for(int i=0;i<5;i++){
       std::cout<<passthrough_limits[i]<<" ";
          }
          std::cout<<'\n';
    }

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = header_camera.str();
    marker2.text = ss2.str();
    set_marker(marker2);

    //////////////////////
    std::stringstream ss3;

    ss3 << "Perpendicular " << perp_ok << " si Paralel " << paral_ok;

    /*
    if (perp_ok && paral_ok)
    {
          ss3<<"Planul e paralel si perpendicular cu podeaua"
    }
    else
    {
      if (perp_ok)
        {
          ss3<<"Planul e perpendicular cu podeaua"
        }
      else
        {
          if (paral_ok)
          {
            ss3<<"Planul e paralel cu podeaua"
          }
          else
          {
            ss3<<"Planul nu are legatura cu podeaua"
          }
        }
    }

    */

    visualization_msgs::Marker marker3;
    marker3.header.frame_id = header_camera.str();
    marker3.text = ss3.str();
    set_marker(marker3);

    ///////////////

    //Publish the data

    pub1_.publish(tempROSMsg);
    pub2_.publish(tempROSMsg2);
    pub3_.publish(tempROSMsg3);

    vis_pub.publish(marker);
    vis2_pub.publish(marker2);
    vis3_pub.publish(marker3);
    vis4_pub.publish(marker_camera);

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

  dynamic_reconfigure::Server<pcl_tutorial::compute_volume_nodeConfig> config_server_;

  double nivel_initial;
  double dividing_number;
  double perpendicular_threshold;
  double parallel_threshold;

  double threshold_x;
  double threshold_y;
  double threshold_z;
  int minimum_nr_points;

  float z_lower_limit;
  float z_upper_limit;
  float y_lower_limit;
  float y_upper_limit;
  float x_lower_limit;
  float x_upper_limit;



  float passthrough_limits[6];

  double selection_camera;

  ros::NodeHandle nh_;
 // ros::Subscriber sub_;

  

  //ros::Subscriber sub_floats;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub3_;

  ros::Publisher vis_pub;
  ros::Publisher vis2_pub;
  ros::Publisher vis3_pub;
  ros::Publisher vis4_pub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_volume_node");

  ComputeVolumeNode vf;

  ros::spin();
}
