#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <ddd_plane_extr/multi_planes_paramConfig.h>

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
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

#include <math.h>

using namespace cv;
using namespace std;
using namespace pcl_msgs;

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
        pub_ = nh_.advertise<PointCloud>("/raw_ransac_plane", 1);
        pub2_ = nh_.advertise<PointCloud>("/ddd_extracted_planes_all_Points", 1);
        pub3_ = nh_.advertise<PointCloud>("/ddd_extracted_planes_gt", 1);

        sub_ = nh_.subscribe("/cloud_pcd", 1, &Planes2Depth::cloudCallback, this);

        // double dist_thr;
        // int max_its;

        config_server_.setCallback(boost::bind(&Planes2Depth::dynReconfCallback, this, _1, _2));

        // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
        ros::NodeHandle private_nh("~");
    }

    ~Planes2Depth() {}

    void dynReconfCallback(ddd_plane_extr::multi_planes_paramConfig &config, uint32_t level)
    {
        _distanceThreshold = config.distanceThreshold;
        _okDistanceThreshold = config.okDistanceThreshold;
        _clusterTolerance = config.clusterTolerance;
        _minClusterSize = config.minClusterSize;
        _maxClusterSize = config.maxClusterSize;
        _maxIterations = config.maxIterations;
        _planesDelay = config.planesDelay;
        _remained_pointcloud = config.remainedPointcloud;
    }

    void cloudCallback(const PointCloud::ConstPtr &cloud_in_msg)
    {
        if (!cloud_in_msg || cloud_in_msg->size() <= 0)
        {
            ROS_WARN("got empty or invalid pointcloud --> ignoring");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_in_msg, *cloud1);
        std::cerr << "Point cloud sizeeeeeeeeeeeeeeeee: " << cloud1->size() << std::endl;

        // cleaning the point cloud of NaNs in order to further apply the Euclidean cluster extraction
        boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
        pcl::removeNaNFromPointCloud(*cloud1, *indices);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud1);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cloud1);
        // ///////////////////////////////////////////////////////////////////////////////////////////

        // Preparing the plane extraction
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(_maxIterations);
        seg.setDistanceThreshold(_distanceThreshold);

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_only(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZ>());

        // Extract inliers
        pcl::ExtractIndices<pcl::PointXYZ> plane_extracter;
        int i = 0, nr_points = (int)cloud1->size();
        std::cerr << "Point cloud size: " << nr_points << std::endl;

        // Stacking all the rectified planes in here for the final reunion
        pcl::PointCloud<pcl::PointXYZ> perfectPlanesCloud;

        // While 30% of the original cloud is still there
        while (cloud1->size() > _remained_pointcloud * nr_points)
        {
            // Extracting the plane using RANSAC
            seg.setInputCloud(cloud1);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
            }

            plane_extracter.setInputCloud(cloud1);
            plane_extracter.setIndices(inliers);
            plane_extracter.setNegative(false);
            // Get the points associated with the planar surface
            plane_extracter.filter(*plane_only);
            std::cerr << "PointCloud representing the planar component: " << plane_only->width * plane_only->height << " data points." << std::endl;

            PointCloud::Ptr plane_only_msg(new PointCloud);
            std::cerr << "------------------------------------" << std::endl;
            std::cerr << "Plane published!" << i << std::endl;
            plane_only_msg->header.stamp = cloud_in_msg->header.stamp;
            plane_only_msg->header.frame_id = "base_link";
            plane_only_msg->points = plane_only->points;
            pub_.publish(plane_only_msg);

            plane_extracter.setNegative(true);
            plane_extracter.filter(*cloud_rest);
            // ////////////////////////////////////////////////////////////////////////

            // Extracting indices of detected clusters
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(plane_only);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(_clusterTolerance); // in meters
            ec.setMinClusterSize(_minClusterSize);
            ec.setMaxClusterSize(_maxClusterSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(plane_only);
            ec.extract(cluster_indices);

            // here should not be more than a single cluster extracted per plane
            int j = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto &idx : it->indices)
                    cloud_cluster->push_back((*plane_only)[idx]); //*
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;

                // publish only the major cluster (aka the plane of interest (POI))
                PointCloud::Ptr cloud_cluster_msg(new PointCloud);
                std::cerr << "------------------------------------" << std::endl;
                std::cerr << "Plane published!" << i << std::endl;
                std::cerr << "Point cloud size!" << cloud1->size() << std::endl;
                cloud_cluster_msg->header.stamp = cloud_in_msg->header.stamp;
                cloud_cluster_msg->header.frame_id = "base_link";
                cloud_cluster_msg->points = cloud_cluster->points;
                pub2_.publish(cloud_cluster_msg);

                j++;
            }
            i++;
            // POI and scraps
            pcl::PointCloud<pcl::PointXYZ>::Ptr scraps_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // now we need to extract the prior detected plane
            pcl::PointIndices::Ptr clusterInliersIndices(new pcl::PointIndices);

            // Extract fInliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> cluster_extracter;
            cluster_extracter.setInputCloud(plane_only);
            cluster_extracter.setIndices(clusterInliersIndices);
            cluster_extracter.setNegative(false); //Removes part_of_cloud but retain the original full_cloud
            cluster_extracter.filter(*plane_only);

            cluster_extracter.setNegative(true); // Removes part_of_cloud from full cloud  and keep the rest
            cluster_extracter.filter(*scraps_cloud);
            // //////////////////////////////////////////////////////////////////////////

            // Preparing the OK plane extraction
            pcl::ModelCoefficients::Ptr okCoefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> okSeg;
            // Optional
            okSeg.setOptimizeCoefficients(true);
            // Mandatory
            okSeg.setModelType(pcl::SACMODEL_PLANE);
            okSeg.setMethodType(pcl::SAC_RANSAC);
            okSeg.setMaxIterations(_maxIterations);
            okSeg.setDistanceThreshold(_okDistanceThreshold);

            okSeg.setInputCloud(plane_only);
            okSeg.segment(*inliers, *okCoefficients);

            // Project all the points from POI to a smooth (and silky) plane #doItLikeZohan
            pcl::PointCloud<pcl::PointXYZ>::Ptr perfectPlane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> perfectProjection;
            perfectProjection.setModelType(pcl::SACMODEL_PLANE);
            perfectProjection.setInputCloud(plane_only);
            perfectProjection.setModelCoefficients(okCoefficients);
            perfectProjection.filter(*perfectPlane);

            // gathering all the rectified planes in the same place for a final concatenation
            pcl::PointCloud<pcl::PointXYZ> tempPerfectPlane;
            if (i == 0)
            {
                perfectPlanesCloud.points = perfectPlane->points;
            }
            else
            {
                tempPerfectPlane.points = perfectPlane->points;
                perfectPlanesCloud += tempPerfectPlane;
            }
            // //////////////////////////////////////////////////////

            // adding the scraps to  the original point cloud
            pcl::PointCloud<pcl::PointXYZ> tempScrapsCloud, tempCloudRest;
            tempScrapsCloud.points = scraps_cloud->points;
            tempCloudRest.points = cloud_rest->points;
            tempCloudRest += tempScrapsCloud;

            cloud_rest->points = tempCloudRest.points;

            // cloud1.swap(cloud_rest);

            // // ##########################################################
            // // #Take only the non-distorted points from the ground floor#
            // // ##########################################################
            // pcl::ModelCoefficients::Ptr okCoefficients(new pcl::ModelCoefficients);
            // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // // Create the segmentation object
            // pcl::SACSegmentation<pcl::PointXYZ> seg;
            // // Optional
            // seg.setOptimizeCoefficients(true);
            // // Mandatory
            // seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            // seg.setMaxIterations(_maxIterations);
            // seg.setMethodType(pcl::SAC_RANSAC);
            // // seg.setDistanceThreshold(_distanceThreshold);
            // seg.setDistanceThreshold(0.001);

            // //because we want a specific plane (X-Z Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
            // Eigen::Vector3f axis1 = Eigen::Vector3f(x_axis, y_axis, z_axis); //y axis
            // seg.setAxis(axis1);
            // seg.setEpsAngle(20.0 * (M_PI / 180.0f)); // plane can be within 10.0 degrees of X-Z plane

            // // Create pointcloud to publish inliers
            // pcl::ExtractIndices<pcl::PointXYZ> extract1;

            // // int original_size(cloud_filtered->size());
            // int n_planes(0);
            // pcl::PointCloud<pcl::PointXYZ> cloudF1;

            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane1(new pcl::PointCloud<pcl::PointXYZ>());
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f1(new pcl::PointCloud<pcl::PointXYZ>());

            // // Fit a plane
            // seg.setInputCloud(cloud1);
            // seg.segment(*inliers, *okCoefficients);

            // // Check result
            // if (inliers->indices.size() == 0)
            // {
            //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            //     // break;
            // }

            // // Extract inliers
            // extract1.setInputCloud(cloud1);
            // extract1.setIndices(inliers);
            // extract1.setNegative(false);
            // // Get the points associated with the planar surface
            // extract1.filter(*cloud_plane1);
            // std::cerr << "PointCloud representing the planar component: " << cloud_plane1->width * cloud_plane1->height << " data points." << std::endl;

            // // Remove the planar inliers, extract the rest
            // extract1.setNegative(true);
            // extract1.filter(*cloud_f1);
            // // cloud_filtered.swap(cloud_f);

            // // Next iteration
            // PointCloud::Ptr final_cloud1(new PointCloud);
            // std::cerr << "------------------------------------" << std::endl;
            // std::cerr << "Nr_planes: " << n_planes << std::endl;
            // final_cloud1->header.frame_id = "base_link";
            // final_cloud1->header.stamp = cloud_in_msg->header.stamp;
            // final_cloud1->points = cloud_plane1->points;
            // pub_.publish(final_cloud1);

            // // ###########################################
            // // #Take all the points from the ground floor#
            // // ###########################################
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::copyPointCloud(*cloud_in_msg, *cloud);
            // // std::cout << std::endl;
            // // std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*
            // // // Create the filtering object: downsample the dataset using a leaf size of 1cm
            // // pcl::VoxelGrid<pcl::PointXYZ> vg;
            // // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);

            // // vg.setInputCloud(cloud);
            // // // float leaf_size = 0.01f;
            // // vg.setLeafSize(leaf_size, leaf_size, leaf_size);
            // // vg.filter(*cloud_filtered_voxel);

            // // std::cout << "PointCloud after filtering has: " << cloud_filtered_voxel->size() << " data points." << std::endl; //*
            // // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            // // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

            // // sor.setInputCloud(cloud_filtered_voxel);
            // // sor.setMeanK(_minClusterSize);
            // // sor.setStddevMulThresh(_maxClusterSize);
            // // sor.setNegative(false);
            // // sor.filter(*cloud_filtered);

            // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            // pcl::PointIndices::Ptr okInliers(new pcl::PointIndices);
            // // Create the segmentation object
            // pcl::SACSegmentation<pcl::PointXYZ> seg2;
            // // Optional
            // seg2.setOptimizeCoefficients(true);
            // // Mandatory
            // seg2.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            // seg2.setMaxIterations(_maxIterations);
            // seg2.setMethodType(pcl::SAC_RANSAC);
            // seg2.setDistanceThreshold(_distanceThreshold);

            // //because we want a specific plane (X-Z Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
            // Eigen::Vector3f axis = Eigen::Vector3f(x_axis, y_axis, z_axis); //y axis
            // seg2.setAxis(axis);
            // seg2.setEpsAngle(_angle * (M_PI / 180.0f)); // plane can be within 30 degrees of X-Z plane

            // // Create pointcloud to publish inliers
            // pcl::ExtractIndices<pcl::PointXYZ> extract;

            // // int original_size(cloud_filtered->size());
            // // int n_planes(0);
            // pcl::PointCloud<pcl::PointXYZ> cloudF;

            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());

            // // Fit a plane
            // seg2.setInputCloud(cloud);
            // seg2.segment(*okInliers, *coefficients);

            // // Check result
            // if (okInliers->indices.size() == 0)
            // {
            //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            //     // break;
            // }

            // // Extract inliers
            // extract.setInputCloud(cloud);
            // extract.setIndices(okInliers);
            // extract.setNegative(false);
            // // Get the points associated with the planar surface
            // extract.filter(*cloud_plane);
            // std::cerr << "PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;

            // // Remove the planar inliers, extract the rest
            // extract.setNegative(true);
            // extract.filter(*cloud_f);
            // // cloud_filtered.swap(cloud_f);

            // // Next iteration
            // PointCloud::Ptr final_cloud(new PointCloud);
            // std::cerr << "------------------------------------" << std::endl;
            // std::cerr << "Nr_planes: " << n_planes << std::endl;
            // final_cloud->header.frame_id = "base_link";
            // final_cloud->header.stamp = cloud_in_msg->header.stamp;
            // final_cloud->points = cloud_plane->points;
            // pub2_.publish(final_cloud);
            // n_planes++;

            // // for (int i = 0; i < cloud_plane->points.size(); i++)
            // // {
            // //     cloud_plane->points[i].y = -1;
            // // }

            // // Project all the points to a plane
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::ProjectInliers<pcl::PointXYZ> proj;
            // proj.setModelType(pcl::SACMODEL_PLANE);
            // proj.setInputCloud(cloud_plane);
            // proj.setModelCoefficients(okCoefficients);
            // proj.filter(*cloud_projected);

            // // Create the final image
            // pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b;
            // // cloud_a = &cloud_f;
            // // pcl::copyPointCloud(cloud_f,cloud_a);
            // cloud_a.points = cloud_f->points;
            // cloud_b.points = cloud_projected->points;
            // cloud_b += cloud_a;

            // PointCloud::Ptr gt_cloud(new PointCloud);
            // std::cerr << "------------------------------------" << std::endl;
            // std::cerr << "Nr_planes: " << n_planes << std::endl;
            // gt_cloud->header.frame_id = "base_link";
            // gt_cloud->header.stamp = cloud_in_msg->header.stamp;
            // gt_cloud->points = cloud_b.points;
            // pub3_.publish(gt_cloud);

            ros::Duration(_planesDelay).sleep();
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
        pcl::PointCloud<pcl::PointXYZ> gt_cloud;
        gt_cloud.points = cloud_rest->points;
        gt_cloud += perfectPlanesCloud;

        PointCloud::Ptr gt_cloud_msg(new PointCloud);
        std::cerr << "------------------------------------" << std::endl;
        // std::cerr << "Nr_planes: " << n_planes << std::endl;
        gt_cloud_msg->header.frame_id = "base_link";
        gt_cloud_msg->header.stamp = cloud_in_msg->header.stamp;
        gt_cloud_msg->points = gt_cloud.points;
        pub3_.publish(gt_cloud_msg);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub2_;
    ros::Publisher pub3_;
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
    dynamic_reconfigure::Server<ddd_plane_extr::multi_planes_paramConfig> config_server_;
    float _distanceThreshold;
    float _okDistanceThreshold;
    float _clusterTolerance;
    float _planesDelay;
    int _minClusterSize;
    int _maxClusterSize;
    float _remained_pointcloud;
    int _maxIterations;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ddd_plane_extr");

    Planes2Depth c2d;

    ros::spin();

    return 0;
}
