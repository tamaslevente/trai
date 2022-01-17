#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/gp3.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/vtk_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/io.h>
#include <string>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace boost;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
//typedef sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZ PointT;
typedef sync_policies::ApproximateTime<Image, Image, PointCloud> MySyncPolicy;

unsigned int cnt = 0;
// char directoryIr[150] = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/multiP6/ir_data/";
char directoryIr[150] = "/home/marian/calibration_ws/ir_rgb_calib/ir_images/";
// char directoryDepth[150] = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/multiP6/depth_data/";
char directoryDepth[150] = "/home/marian/calibration_ws/ir_rgb_calib/rgb_images/";
// char directoryPcd[150] = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/multiP6/pcd_data/";
char directoryPcd[150] = "/home/marian/calibration_ws/ir_rgb_calib/";

void callback(const ImageConstPtr &ir, const ImageConstPtr &depth, const PointCloud::ConstPtr &cloud_in)
{
    //ROS_INFO_STREAM("Data Arrival\n");

    cv_bridge::CvImagePtr img_ptr_ir;
    // cv_bridge::CvImagePtr img_ptr_rgb;
    cv_bridge::CvImagePtr img_ptr_depth;
    

    img_ptr_ir = cv_bridge::toCvCopy(*ir, sensor_msgs::image_encodings::TYPE_16UC1);
    // img_ptr_rgb = cv_bridge::toCvCopy(*rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    img_ptr_depth = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat &mat_ir = img_ptr_ir->image;

    cv::Mat &mat_depth = img_ptr_depth->image;


    char file_ir[150];

    char file_pcd[150];
    char file_depth[150];
    sprintf(file_ir, "%s%04d_ir.png", directoryIr, cnt);

    sprintf(file_depth, "%s%04d_depth.png", directoryDepth, cnt);
    sprintf(file_pcd, "%s%04d_pcd.pcd", directoryPcd, cnt);

    cv::imwrite(file_ir, mat_ir);

    cv::imwrite(file_depth, mat_depth);
    pcl::io::savePCDFileASCII(file_pcd, *cloud_in);
    std::cout << "Input data number " << int(cnt) << " is saved" << std::endl;

    cnt++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saving_images");
    ros::NodeHandle nh_;
    message_filters::Subscriber<Image> ir_sub(nh_, "ir_input", 1000);
    message_filters::Subscriber<Image> depth_sub(nh_, "depth_input", 1000);
    message_filters::Subscriber<PointCloud> pcd_sub(nh_, "point_cloud_in", 1000);
    // message_filters::Subscriber<Image> rgb_sub(nh_, "rgb_input", 1000);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ir_sub, depth_sub, pcd_sub);
    //TimeSynchronizer<Image, Image, PointCloud, Image> sync(ir_sub, depth_sub, pcd_sub, rgb_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    ros::spin();
    return 0;
}