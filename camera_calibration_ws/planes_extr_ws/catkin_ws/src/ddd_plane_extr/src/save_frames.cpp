#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <limits>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <thread>
#include <visualization_msgs/Marker.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
//typedef sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZ PointT;
typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

unsigned int cnt = 0;
char directory[100] = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/ir/";

void callback(const ImageConstPtr &depth, const ImageConstPtr &rgb)
{
    ROS_INFO_STREAM("Data Arrival\n");

    // cv_bridge::CvImagePtr img_ptr_ir;
    cv_bridge::CvImagePtr img_ptr_rgb;
    cv_bridge::CvImagePtr img_ptr_depth32;
    cv_bridge::CvImagePtr img_ptr_depth;
    // std::string enci=ir->encoding;
    // std::cout<<enci<<std::endl;
    // img_ptr_ir = cv_bridge::toCvCopy(*ir, sensor_msgs::image_encodings::TYPE_16UC1);
    img_ptr_rgb = cv_bridge::toCvCopy(*rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    img_ptr_depth32 = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat &mat_depth = img_ptr_depth32->image;

    // double minVal;
    // double maxVal;
    // Point minLoc;
    // Point maxLoc;

    // cv::Mat mat_depth;
    // cv::normalize(mat_depth32, mat_depth, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
    // mat_depth32=mat_depth32*1000;
    // mat_depth32.convertTo(mat_depth,CV_16UC1);
    // cv::minMaxLoc(mat_depth32, &minVal, &maxVal);

    
    // cout << "min val: " << minVal << endl;
    // cout << "max val: " << maxVal << endl;
    // cout << "center32: " << mat_depth32.at<float>(320,240) << endl;
    // cout << "center: " << mat_depth.at<uint16_t>(320,240) << endl;

    // cv::Mat &mat_ir = img_ptr_ir->image;
    // cv::Mat mat_ir_contrast = cv::Mat::zeros(mat_ir.rows, mat_ir.cols, CV_16UC1);
    cv::Mat &mat_rgb = img_ptr_rgb->image;
    // cv::Mat &mat_depth = img_ptr_depth->image;
    // cv::convertScaleAbs(mat_ir, mat_ir_contrast, 0.15, 0.0);

    // cv::convertScaleAbs(mat_depth, mat_depth, 0.03, 1.0);
    // cv::Mat zerochannel = cv::Mat::zeros(cv::Size(mat_depth.rows, mat_depth.cols), CV_16U);
    // cv::Mat outputd3 = cv::Mat::zeros(mat_depth.rows, mat_depth.cols, CV_16UC3);
    // cv::Mat outputd2i = cv::Mat::zeros(mat_depth.rows, mat_depth.cols, CV_16UC3);
    // char file_outputd3[100];
    // char file_outputd2i[100];

    // cv::Mat imagesd3[3] = {mat_depth, mat_depth, mat_depth};
    // cv::Mat imagesd2i[3] = {mat_ir, mat_depth, mat_depth};
    // sprintf(file_outputd3, "%sdepth3/%05d_depth3.png", directory, cnt);
    // sprintf(file_outputd2i, "%sdepth2ir/%05d_depth2ir.png", directory, cnt);

    // int dimsd3[3] = {2, mat_depth.rows, mat_depth.cols};
    // int dimsd2i[3] = {2, mat_depth.rows, mat_depth.cols};
    // cv::Mat joinedd3(3, dimsd3, CV_16U);
    // cv::Mat joinedd2i(3, dimsd2i, CV_16U);
    // for (int i = 0; i < 3; i++)
    // {
    //     uint16_t *ptrd3 = &joinedd3.at<uint16_t>(i, 0, 0);                            // pointer to first element of slice i
    //     cv::Mat destinationd3(mat_depth.rows, mat_depth.cols, CV_32S, (void *)ptrd3); // no data copy, see documentation
    //     imagesd3[i].copyTo(destinationd3);

    //     uint16_t *ptrd2i = &joinedd2i.at<uint16_t>(i, 0, 0);                            // pointer to first element of slice i
    //     cv::Mat destinationd2i(mat_depth.rows, mat_depth.cols, CV_32S, (void *)ptrd2i); // no data copy, see documentation
    //     imagesd2i[i].copyTo(destinationd2i);
    // }

    // for (int x = 0; x < imagesd3[0].rows; x++)
    // {
    //     for (int y = 0; y < imagesd3[0].cols; y++)
    //     {
    //         outputd3.at<cv::Vec3s>(x, y)[2] = imagesd3[0].at<unsigned short>(x, y);
    //         outputd3.at<cv::Vec3s>(x, y)[1] = imagesd3[1].at<unsigned short>(x, y);
    //         outputd3.at<cv::Vec3s>(x, y)[0] = imagesd3[2].at<unsigned short>(x, y);

    //         outputd2i.at<cv::Vec3s>(x, y)[2] = imagesd2i[0].at<unsigned short>(x, y);
    //         outputd2i.at<cv::Vec3s>(x, y)[1] = imagesd2i[1].at<unsigned short>(x, y);
    //         outputd2i.at<cv::Vec3s>(x, y)[0] = imagesd2i[2].at<unsigned short>(x, y);
    //     }
    // }

    // char file_ir[100];
    // char file_ir_contrast[100];
    char file_rgb[100];
    // char file_pcd[100];
    char file_depth[100];
    // sprintf(file_ir, "%sir/%05d_ir.png", directory, cnt);
    // sprintf(file_ir_contrast, "%s%04d_ir_contrast.png", directory, cnt);
    sprintf(file_rgb, "%srgb/%05d_rgb.png", directory, cnt);
    // sprintf(file_pcd, "%s%04d_pcd.pcd", directory, cnt);
    sprintf(file_depth, "%sdepth/%05d_ir.png", directory, cnt);

    // cv::imwrite(file_ir, mat_ir);
    // cv::imwrite(file_ir_contrast, mat_ir_contrast);
    cv::imwrite(file_rgb, mat_rgb);
    cv::imwrite(file_depth, mat_depth);
    // pcl::io::savePCDFile(file_pcd, *cloud_in, true);
    std::cout << "Input data number " << int(cnt) << " is saved" << std::endl;

    // cv::imwrite(file_outputd3, outputd3);
    //  cv::imwrite(file_outputd2i, outputd2i);
    // std::cout << "Depth+ir is saved" << std::endl;
    cnt++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normalrgb");
    ros::NodeHandle nh_;
    message_filters::Subscriber<Image> ir_sub(nh_, "ir_input", 1000);
    // message_filters::Subscriber<Image> depth_sub(nh_, "depth_input", 1);
    // message_filters::Subscriber<PointCloud> pcd_sub(nh_, "point_cloud_in", 1000);
    message_filters::Subscriber<Image> rgb_sub(nh_, "rgb_input", 1000);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), ir_sub, rgb_sub);
    //TimeSynchronizer<Image, Image, PointCloud, Image> sync(ir_sub, depth_sub, pcd_sub, rgb_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
}
