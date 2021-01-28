// #include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <ddd_plane_extr/planes_paramConfig.h>

// Include opencv2
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// #include <cv_bridge/cv_bridge.h>
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
#include <pcl/filters/project_inliers.h>

#include <iostream>
#include <experimental/filesystem>
#include <iterator>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace cv;
using namespace std;
using namespace pcl_msgs;
// namespace fs = boost::filesystem;
namespace fs = std::experimental::filesystem;

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
        // pub_ = nh_.advertise<PointCloud>("/ddd_extracted_planes", 1);
        // pub2_ = nh_.advertise<PointCloud>("/ddd_extracted_planes_all_Points", 1);
        // pub3_ = nh_.advertise<PointCloud>("/ddd_extracted_gt", 1);
        // pub4_ = nh_.advertise<PointCloud>("/ddd_extracted_plane_gt", 1);

        // sub_ = nh_.subscribe("/pico_pcloud_rect", 1, &Planes2Depth::cloudCallback, this);

        // config_server_.setCallback(boost::bind(&Planes2Depth::dynReconfCallback, this, _1, _2));

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
        ros::NodeHandle private_nh("~");
        string saveDirPcd = "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/training_data/pcd_gt/"; // BEWARE of that last / !!! You don't want to forget that, trust me!
        string saveDirDepth = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/depth_gt/"; // BEWARE of that last / !!! You don't want to forget that, trust me!
        long int np = saveDirPcd.length();
        char saveDirPcdArr[np+1];
        long int nd = saveDirDepth.length();
        char saveDirDepthArr[nd+1];
        strcpy(saveDirPcdArr, saveDirPcd.c_str());
        strcpy(saveDirDepthArr, saveDirDepth.c_str());
        if (private_nh.getParam("pcd_folder", _param))
        {
            ROS_INFO("Got param: %s", _param.c_str());
            int i = 0;
            fs::path p(_param.c_str());
            typedef vector<fs::path> vec;             // store paths,
            vec v;                                // so we can sort them later
            copy(fs::directory_iterator(p), fs::directory_iterator(), back_inserter(v));
            sort(v.begin(), v.end());
            for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
            {
                // string pcd_file = *it.string();
                cout << "pcd_file:" << *it << '\n';
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(*it, *cloud) == -1) //* load the file
                {
                    PCL_ERROR("Couldn't read file %s \n",it);
                }
                // cout << entry.path() << '\n';
                std::cout << "Loaded "
                          << cloud->width * cloud->height
                          << " data points"
                          << std::endl;
                string saveDirP(saveDirPcdArr);
                char numberDirPcdArr[6];
                sprintf(numberDirPcdArr,"%05d",i);
                string pcdFilename = saveDirP + numberDirPcdArr + ".pcd";
                string saveDirD(saveDirDepthArr);
                char numberDirDepthArr[6];
                sprintf(numberDirDepthArr,"%05d",i);
                string depthFilename = saveDirD + numberDirDepthArr + ".png";
                getFloorGT(cloud,pcdFilename,depthFilename);
                i++;
            }
        }
        else
        {
            ROS_ERROR("Failed to get param 'pcd_folder' ");
        }
    }

    ~Planes2Depth() {}

    // void dynReconfCallback(ddd_plane_extr::planes_paramConfig &config, uint32_t level)
    // {
    //     _distanceThreshold = config.distanceThreshold;
    //     _planesDelay = config.planesDelay;
    //     // _max_planes = config.max_planes;
    //     _MeanK = config.MeanK;
    //     _StddevMulThresh = config.StddevMulThresh;
    //     _maxIterations = config.maxIterations;
    //     _angle = config.angle;
    // }

    PointCloud::Ptr *extractPlanes(PointCloud::Ptr inputCloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, float distanceThreshold, float angle, int maxIterations)
    {
        static PointCloud::Ptr pointClouds[2];

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMaxIterations(maxIterations);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distanceThreshold);

        //because we want a specific plane (X-Z Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
        Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0); //y axis
        seg.setAxis(axis);
        seg.setEpsAngle(angle * (M_PI / 180.0f)); // plane can be within 10.0 degrees of X-Z plane

        // Create pointcloud to publish inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr inversed_cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

        // Fit a plane
        seg.setInputCloud(inputCloud);
        seg.segment(*inliers, *coefficients);

        // Check result
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            // break;
        }

        // Extract inliers
        extract.setInputCloud(inputCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        // Let only the points that are not in the planar surface
        extract.setNegative(true);
        extract.filter(*inversed_cloud_plane);

        pointClouds[0] = cloud_plane;
        pointClouds[1] = inversed_cloud_plane;

        return pointClouds;
    }

    // PointCloud::Ptr voxelFilter(){
    //     // std::cout << std::endl;
    //     // std::cout << "PointCloud before filtering has: " << cloud1->size() << " data points." << std::endl; //*
    //     // // Create the filtering object: downsample the dataset using a leaf size of 1cm
    //     // pcl::VoxelGrid<pcl::PointXYZ> vg1;
    //     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel1(new pcl::PointCloud<pcl::PointXYZ>);

    //     // vg1.setInputCloud(cloud1);
    //     // float leaf_size = 0.01f;
    //     // vg1.setLeafSize(leaf_size, leaf_size, leaf_size);
    //     // vg1.filter(*cloud_filtered_voxel1);

    //     // std::cout << "PointCloud after filtering has: " << cloud_filtered_voxel1->size() << " data points." << std::endl; //*
    //     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
    //     // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;

    //     // sor1.setInputCloud(cloud_filtered_voxel1);
    //     // sor1.setMeanK(_MeanK);
    //     // sor1.setStddevMulThresh(_StddevMulThresh);
    //     // sor1.setNegative(false);
    //     // sor1.filter(*cloud_filtered1);
    // }

    void getFloorGT(PointCloud::Ptr inputCloud, string pcd_file_name, string depth_file_name)
    {
        if (!inputCloud || inputCloud->size() <= 0)
        {
            ROS_WARN("got empty or invalid pointcloud --> ignoring");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(inputCloud);
        // pcl::copyPointCloud(*cloud_in_msg, *cloud1);

        // ##########################################################
        // #Take only the non-distorted points from the ground floor#
        // ##########################################################
        pcl::ModelCoefficients::Ptr okCoefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Extract the base point for floor, here we are interested in getting the coefficients of these points
        PointCloud::Ptr *pointClouds = extractPlanes(cloud1, inliers, okCoefficients, 0.03, 23.0, 1000);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane1(*pointClouds);

        // Publish pointcloud with tese points
        PointCloud::Ptr final_cloud1(new PointCloud);
        std::cerr << "------------------------------------" << std::endl;
        // std::cerr << "Nr_planes: " << n_planes << std::endl;
        final_cloud1->header.frame_id = "base_link";
        final_cloud1->header.stamp = inputCloud->header.stamp;
        final_cloud1->points = cloud_plane1->points;
        // pub_.publish(final_cloud1);

        // ###########################################
        // #Take all the points from the ground floor#
        // ###########################################
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(inputCloud);
        // pcl::copyPointCloud(*inputCloud, *cloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr okInliers(new pcl::PointIndices);

        // Extract as many points as you can from the ground floor (that is why we'll use larger numbers:
        //         _distanceThreshold: 0.13 (meters) (pretty huge, I know!)
        pointClouds = extractPlanes(cloud, okInliers, coefficients, 0.13, 20.0, 1000);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(*pointClouds);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane(*(pointClouds + 1));

        // Publish this point cloud
        PointCloud::Ptr final_cloud(new PointCloud);
        std::cerr << "------------------------------------" << std::endl;
        final_cloud->header.frame_id = "base_link";
        final_cloud->header.stamp = inputCloud->header.stamp;
        final_cloud->points = cloud_plane->points;
        // pub2_.publish(final_cloud);

        // Project all the points to a plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud_plane);
        proj.setModelCoefficients(okCoefficients);
        proj.filter(*cloud_projected);

        // Publish the point cloud
        PointCloud::Ptr gt_plane(new PointCloud);
        std::cerr << "------------------------------------" << std::endl;
        gt_plane->header.frame_id = "base_link";
        gt_plane->header.stamp = inputCloud->header.stamp;
        gt_plane->points = cloud_projected->points;
        // pub4_.publish(gt_plane);

        // Create the final image
        pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b;
        cloud_a.points = cloud_no_plane->points;
        cloud_b.points = cloud_projected->points;
        cloud_b += cloud_a;

        PointCloud::Ptr gt_cloud(new PointCloud);
        std::cerr << "------------------------------------" << std::endl;
        gt_cloud->header.frame_id = "base_link";
        gt_cloud->header.stamp = inputCloud->header.stamp;
        gt_cloud->points = cloud_b.points;
        gt_cloud->is_dense = false;
        gt_cloud->points.resize(inputCloud->width * inputCloud->height);
        gt_cloud->width = 1;
        gt_cloud->height = gt_cloud->points.size();
        // pub3_.publish(gt_cloud);

        //##########################//
        // save the pcd if you want //
        //##########################//
        // pcl::io::savePCDFileASCII (pcd_file_name, *gt_cloud);
        // std::cerr << "Saved " << gt_cloud->size() << " data points to"<< pcd_file_name << std::endl;

        // ***********************
        // **CONVERSION TO DEPTH**
        // ***********************

        cv_image = Mat(height_, width_, CV_32FC1, 0.0); //Scalar(std::numeric_limits<float>::max()));
        int minRange = 0; // this is the smallest distance at which the camera can measure depending on the mode (near, far, etc.)

        for (int i = 0; i < gt_cloud->points.size(); i++)
        {
            if (gt_cloud->points[i].z == gt_cloud->points[i].z)
            {
                if (gt_cloud->points[i].z != 0.0)
                {
                    z = gt_cloud->points[i].z * 1000.0;
                    u = (gt_cloud->points[i].x * 1000.0 * focalX_) / z;
                    v = (gt_cloud->points[i].y * 1000.0 * focalY_) / z;
                    pixel_pos_x = (int)(u + centerX_);
                    pixel_pos_y = (int)(v + centerY_);

                    if (pixel_pos_x > (width_ - 1))
                    {
                        pixel_pos_x = width_ - 1;
                    }
                    else if (pixel_pos_x < 0)
                    {
                        pixel_pos_x = -pixel_pos_x;
                    }
                    if (pixel_pos_y > (height_ - 1))
                    {
                        pixel_pos_y = height_ - 1;
                    }
                    else if (pixel_pos_y < 0)
                    {
                        pixel_pos_y = -pixel_pos_y;
                    }
                }
                else
                {
                    pixel_pos_x = 0;
                    pixel_pos_y = 0;
                    z = 0.0;
                }

                cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z - minRange;
            }
        }

        //###########################//
        //  saving the depth images  //
        //###########################//
        cv_image.convertTo(cv_image, CV_16UC1);
        cv::imwrite(depth_file_name, cv_image);
        std::cerr << "Saved " << gt_cloud->size() << " data points to"<< depth_file_name << std::endl;

    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub2_;
    ros::Publisher pub3_;
    ros::Publisher pub4_;

    // image_transport::Publisher pub2_;
    // calibration parameters
    // double K[9] = {385.8655956930966, 0.0, 342.3593021849471,
    //                0.0, 387.13463636528166, 233.38372018194542,
    //                0.0, 0.0, 1.0};

    // EEPROM parameters
    // double K[9] = {386.804, 0.0, 341.675,
    //                0.0, 384, 238.973,
    //                0.0, 0.0, 1.0};
    
    // Pico parameters
    double K[9] = {460.585, 0.0, 334.080, 
                    0.0, 460.268, 169.807, 
                    0.0, 0.0, 1.0};

    double centerX_ = K[2];
    double centerY_ = K[5];
    double focalX_ = K[0];
    double focalY_ = K[4];
    // int height_ = 480;
    int height_ = 360; // only for pico
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
    string _param;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ddd_plane_extr");

    Planes2Depth c2d;

    // ros::spin();

    return 0;
}
