#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <ddd_plane_extr/planes_paramConfig.h>

// Include opencv2
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/publisher.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace pcl_msgs;
// namespace plt = matplotlibcpp;

class Planes2Depth
{
public:
    // typedef pcl::PointXYZRGB Point;
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    /*
   * constructor
   * setup which topics are passed out (advertise) and to which topics are listend (subscribe)
   */
    Planes2Depth() : it_(nh_)
    {
        pub_ = nh_.advertise<PointCloud>("/ddd_extracted_planes", 1);

        sub_ = nh_.subscribe("/cloud_pcd", 1, &Planes2Depth::cloudCallback, this);

        // double dist_thr;
        // int max_its;

        config_server_.setCallback(boost::bind(&Planes2Depth::dynReconfCallback, this, _1, _2));

        // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
        ros::NodeHandle private_nh("~");
    }

    ~Planes2Depth() {}

    void dynReconfCallback(ddd_plane_extr::planes_paramConfig &config, uint32_t level)
    {
        _distanceThreshold = config.distanceThreshold;
        _planesDelay = config.planesDelay;
        // _max_planes = config.max_planes;
        _MeanK = config.MeanK;
        _StddevMulThresh = config.StddevMulThresh;
        _maxIterations = config.maxIterations;
        _angle = config.angle;
    }

    void cloudCallback(const PointCloud::ConstPtr &cloud_in_msg)
    {
        if (!cloud_in_msg || cloud_in_msg->size() <= 0)
        {
            ROS_WARN("got empty or invalid pointcloud --> ignoring");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_in_msg, *cloud);
        std::cout << std::endl;
        std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);

        vg.setInputCloud(cloud);
        float leaf_size = 0.01f;
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_filtered_voxel);

        std::cout << "PointCloud after filtering has: " << cloud_filtered_voxel->size() << " data points." << std::endl; //*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

        sor.setInputCloud(cloud_filtered_voxel);
        sor.setMeanK(_MeanK);
        sor.setStddevMulThresh(_StddevMulThresh);
        sor.setNegative(false);
        sor.filter(*cloud_filtered);

        // Take only the ground floor

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        // seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMaxIterations(_maxIterations);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(_distanceThreshold);

        //because we want a specific plane (X-Z Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
        Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0); //y axis
        seg.setAxis(axis);
        seg.setEpsAngle(_angle * (M_PI / 180.0f)); // plane can be within 30 degrees of X-Z plane

        // Create pointcloud to publish inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // int original_size(cloud_filtered->size());
        int n_planes(0);
        pcl::PointCloud<pcl::PointXYZ> cloudF;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());
        // while (_max_planes > n_planes)
        // {

            // Fit a plane
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);

            // Check result
            if (inliers->indices.size() == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                // break;
            }

            // Extract inliers
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);
            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            std::cerr << "PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            // cloud_filtered.swap(cloud_f);

            // Next iteration
            PointCloud::Ptr final_cloud(new PointCloud);
            std::cerr << "------------------------------------" << std::endl;
            std::cerr << "Nr_planes: " << n_planes << std::endl;
            final_cloud->header.frame_id = "base_link";
            final_cloud->header.stamp = cloud_in_msg->header.stamp;
            // final_cloud->size() = cloud_plane->size();
            final_cloud->points = cloud_plane->points;
            ros::Duration(_planesDelay).sleep();
            pub_.publish(final_cloud);
            n_planes++;
        // }

        // ***********************
        // **CONVERSION TO DEPTH**
        // ***********************

        // cv_image = Mat(height_, width_, CV_32FC1, 0.0); //Scalar(std::numeric_limits<float>::max()));
        // int minRange = 0; // this is the smallest distance at which the camera can measure depending on the mode (near, far, etc.)

        // for (int i = 0; i < cloud_in_msg->points.size(); i++)
        // {
        //     if (cloud_in_msg->points[i].z == cloud_in_msg->points[i].z)
        //     {
        //         if (cloud_in_msg->points[i].z != 0.0)
        //         {
        //             z = cloud_in_msg->points[i].z * 1000.0;
        //             u = (cloud_in_msg->points[i].x * 1000.0 * focalX_) / z;
        //             v = (cloud_in_msg->points[i].y * 1000.0 * focalY_) / z;
        //             pixel_pos_x = (int)(u + centerX_);
        //             pixel_pos_y = (int)(v + centerY_);

        //             if (pixel_pos_x > (width_ - 1))
        //             {
        //                 pixel_pos_x = width_ - 1;
        //             }
        //             else if (pixel_pos_x < 0)
        //             {
        //                 pixel_pos_x = -pixel_pos_x;
        //             }
        //             if (pixel_pos_y > (height_ - 1))
        //             {
        //                 pixel_pos_y = height_ - 1;
        //             }
        //             else if (pixel_pos_y < 0)
        //             {
        //                 pixel_pos_y = -pixel_pos_y;
        //             }
        //         }
        //         else
        //         {
        //             pixel_pos_x = 0;
        //             pixel_pos_y = 0;
        //             z = 0.0;
        //         }

        //         cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z - minRange;
        //     }
        // }

        // cv_image.convertTo(cv_image, CV_16UC1);

        // // imshow("Display depth from point cloud", cv_image);
        // waitKey(3);

        // // imwrite("depth_from_pcd.png",cv_image);
        // sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
        // pub_.publish(output_image);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    // image_transport::Publisher pub2_;
    // calibration parameters
    // double K[9] = {385.8655956930966, 0.0, 342.3593021849471,
    //                0.0, 387.13463636528166, 233.38372018194542,
    //                0.0, 0.0, 1.0};

    // EEPROM parameters
    double K[9] = {386.804, 0.0, 341.675,
                   0.0, 384, 238.973,
                   0.0, 0.0, 1.0};

    double centerX_ = K[2];
    double centerY_ = K[5];
    double focalX_ = K[0];
    double focalY_ = K[4];
    int height_ = 480;
    int width_ = 640;
    int pixel_pos_x, pixel_pos_y;
    float z, u, v;
    Mat cv_image;
    std::vector<Point2d> imagePoints;
    dynamic_reconfigure::Server<ddd_plane_extr::planes_paramConfig> config_server_;
    float _distanceThreshold;
    int _max_planes;
    float _planesDelay;
    int _MeanK;
    double _StddevMulThresh;
    float _angle;
    int _maxIterations;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ddd_plane_extr");

    Planes2Depth c2d;

    ros::spin();

    return 0;
}