/*
 * voxel_filter.cpp
 *
 *  Created on: 06.09.2013
 *      Author: goa
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ppfplane/voxel_filter_nodeConfig.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

// includes
#include <iostream>
#include <fstream>
#include <string> 
#include "definitions.h"
// libraries
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <CLI/CLI.hpp>
// classes
#include "Timer.h"
#include "Plane.h"
#include "graph/PlaneGraph.h"
#include "graph/ParallelPlaneGraph.h"
#include "PPF/PairDetector.h"
// function includes
//#include "io/load_ply_cloud.h"
//#include "visualize/pcshow.h"

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

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include "std_msgs/String.h"



class LineDetectNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  LineDetectNode()
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_final",1);
    sub_ = nh_.subscribe ("/norm_out", 1,  &LineDetectNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&LineDetectNode::dynReconfCallback, this, _1, _2));
  }

  ~LineDetectNode() {}

  void
  dynReconfCallback(ppfplane::voxel_filter_nodeConfig &config, uint32_t level)
  {

  }

  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::PointCloud<pcl::PointNormal> cloud_Test;
    pcl::fromROSMsg(*cloud_msg, cloud_Test);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointNormal>);
    *cloudPTR = cloud_Test;

    std::cout<<cloudPTR->size()<<"\n";
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<ppfplane::voxel_filter_nodeConfig> config_server_;



};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "voxel_filter_node");

  LineDetectNode vf;

  ros::spin();
}

