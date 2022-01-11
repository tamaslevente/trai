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
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>


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
        pub2_ = nh_.advertise<PointCloud>("/loose_plane", 1);
        pub3_ = nh_.advertise<PointCloud>("/gt_planes", 1);
        pub4_ = nh_.advertise<PointCloud>("/scraps_cloud", 1);
        pub5_ = nh_.advertise<PointCloud>("/perfect_plane", 1);
        pub6_ = nh_.advertise<PointCloud>("/remained_cloud_after_a_POI_extraction", 1);
        pub7_ = nh_.advertise<PointCloud>("/cluster_cloud", 1);
        pub8_ = nh_.advertise<PointCloud>("/the_big_plane", 1);
        pub9_ = nh_.advertise<PointCloud>("/cropped_polygon", 1);

        sub_ = nh_.subscribe("/pico_pcloud", 1, &Planes2Depth::cloudCallback, this);

        config_server_.setCallback(boost::bind(&Planes2Depth::dynReconfCallback, this, _1, _2));

        // "~" means, that the node hand is opened within the private namespace (to get the "own" parameters)
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr only_points(new pcl::PointCloud<pcl::PointXYZ>);
        if (_firstTime == 1)
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>("my_full_custom_pcdmultiP5.pcd", *only_points) == -1) //* load the file
            {
                PCL_ERROR("Couldn't read file test_pcd.pcd \n");
            }
            std::cout << "Loaded "
                      << only_points->width * only_points->height
                      << " data points for plane completion purposes... ;)"
                      << std::endl;
            _firstTime = 1;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_in_msg, *cloud1);

        std::cerr << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#" << std::endl;
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZ>());

        int i = 0, nr_points = (int)cloud1->size(); ////////////////// ?????????????????????????
        std::cerr << "Point cloud size: " << nr_points << std::endl;

        // Stacking all the rectified planes in here for the final reunion
        pcl::PointCloud<pcl::PointXYZ> perfectPlanesCloud;

        pcl::PointCloud<pcl::PointXYZ>::Ptr only_points_copy(new pcl::PointCloud<pcl::PointXYZ>);
        // While 30% of the original cloud is still there
        while (cloud1->size() > _remained_pointcloud * nr_points)
        {
            // Extract inliers
            pcl::ExtractIndices<pcl::PointXYZ> plane_extracter;

            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_only(new pcl::PointCloud<pcl::PointXYZ>());
            // Extracting the plane using RANSAC
            seg.setInputCloud(cloud1);
            seg.segment(*inliers, *coefficients);

            std::vector<int> ransac_plane_indices;
            ransac_plane_indices = inliers->indices;

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

            // ////////////////////////////////////////////////////////////////////////

            // Extracting indices of detected clusters for identifying the only plane
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(plane_only);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(_clusterTolerance); // in meters
            ec.setMinClusterSize(_minClusterSize);
            ec.setMaxClusterSize(_maxClusterSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(plane_only);

            std::vector<int> POIIndices;

            ec.extract(cluster_indices);

            // here should not be more than a single cluster extracted per plane
            int j = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto &idx : it->indices)
                {
                    cloud_cluster->push_back((*plane_only)[idx]); //*
                    POIIndices.push_back(ransac_plane_indices[idx]);
                }
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
                pub7_.publish(cloud_cluster_msg);
                j++;
            }
            i++;

            // POI and scraps
            pcl::PointCloud<pcl::PointXYZ>::Ptr scraps_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // now we need to extract the prior detected plane
            pcl::PointIndices::Ptr clusterInliersIndices(new pcl::PointIndices);
            clusterInliersIndices->indices = POIIndices;

            // Extract Inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> cluster_extracter;
            cluster_extracter.setInputCloud(cloud1);
            cluster_extracter.setIndices(clusterInliersIndices);
            cluster_extracter.setNegative(false); // Keep only the inliers (i.e. the plane)
            cluster_extracter.filter(*plane_only);

            cluster_extracter.setNegative(true); // Remove the inliers (i.e. remove only the plane and keep the rest)
            cluster_extracter.filter(*scraps_cloud);
            // //////////////////////////////////////////////////////////////////////////

            // publish only the major cluster (aka the plane of interest (POI))
            PointCloud::Ptr cloud_cluster_msg(new PointCloud);
            std::cerr << "------------------------------------" << std::endl;
            std::cerr << "Plane published!" << i << std::endl;
            std::cerr << "Plane only size!" << plane_only->size() << std::endl;
            cloud_cluster_msg->header.stamp = cloud_in_msg->header.stamp;
            cloud_cluster_msg->header.frame_id = "base_link";
            cloud_cluster_msg->points = plane_only->points;
            pub2_.publish(cloud_cluster_msg);

            PointCloud::Ptr scraps_cloud_msg(new PointCloud);
            std::cerr << "------------------------------------" << std::endl;
            std::cerr << "Scraps cloud size!" << scraps_cloud->size() << std::endl;
            scraps_cloud_msg->header.stamp = cloud_in_msg->header.stamp;
            scraps_cloud_msg->header.frame_id = "base_link";
            scraps_cloud_msg->points = scraps_cloud->points;
            pub4_.publish(scraps_cloud_msg);

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

            // Project many points on the same plane for creating a perfect (and hopefully big) plane
            // We need a copy, because otherwise only_points is going to be empty after the first iteration
            pcl::copyPointCloud(*only_points, *only_points_copy);
            pcl::PointCloud<pcl::PointXYZ>::Ptr theBigPlane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> bigPlaneProjection;
            bigPlaneProjection.setModelType(pcl::SACMODEL_PLANE);
            bigPlaneProjection.setInputCloud(only_points_copy);
            bigPlaneProjection.setModelCoefficients(okCoefficients);
            bigPlaneProjection.filter(*theBigPlane);

            // Project all the points from POI to a smooth (and silky) plane .....................................................................................................................#doItLikeZohan
            pcl::PointCloud<pcl::PointXYZ>::Ptr perfectPlane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> perfectProjection;
            perfectProjection.setModelType(pcl::SACMODEL_PLANE);
            perfectProjection.setInputCloud(plane_only);
            perfectProjection.setModelCoefficients(okCoefficients);
            perfectProjection.filter(*perfectPlane);

            // The concave hull extraction
            pcl::ConcaveHull<pcl::PointXYZ> hull;
            hull.setInputCloud(perfectPlane);
            hull.setDimension(2);
            std::cout << hull.getDimension() << std::endl;
            hull.setAlpha(0.01); // pretty sensible
            std::vector<pcl::Vertices> polygons;                                                  //Set the vector of pcl::Vertices type to save the convex hull vertices
            pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>); //This point cloud is used to describe the shape of the convex hull
            hull.reconstruct(*surface_hull, polygons);

            pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::CropHull<pcl::PointXYZ> bb_filter;
            bb_filter.setDim(2);
            bb_filter.setInputCloud(theBigPlane);
            bb_filter.setHullIndices(polygons);
            bb_filter.setHullCloud(surface_hull);
            bb_filter.filter(*objects);
            std::cout << objects->size() << std::endl;

            PointCloud::Ptr the_big_plane_msg(new PointCloud);
            std::cerr << "################################" << std::endl;
            std::cerr << "Perfect plane size!" << theBigPlane->size() << std::endl;
            the_big_plane_msg->header.stamp = cloud_in_msg->header.stamp;
            the_big_plane_msg->header.frame_id = "base_link";
            the_big_plane_msg->points = theBigPlane->points;
            pub8_.publish(the_big_plane_msg);

            PointCloud::Ptr perfect_plane_msg(new PointCloud);
            std::cerr << "################################" << std::endl;
            std::cerr << "Perfect plane size!" << perfectPlane->size() << std::endl;
            perfect_plane_msg->header.stamp = cloud_in_msg->header.stamp;
            perfect_plane_msg->header.frame_id = "base_link";
            perfect_plane_msg->points = perfectPlane->points;
            pub5_.publish(perfect_plane_msg);

            PointCloud::Ptr objects_msg(new PointCloud);
            std::cerr << "################################" << std::endl;
            std::cerr << "Perfect plane size!" << objects->size() << std::endl;
            objects_msg->header.stamp = cloud_in_msg->header.stamp;
            objects_msg->header.frame_id = "base_link";
            objects_msg->points = objects->points;
            pub9_.publish(objects_msg);

            // gathering all the rectified planes in the same place for a final concatenation
            pcl::PointCloud<pcl::PointXYZ> tempPerfectPlane;
            tempPerfectPlane.points = perfectPlane->points;

            pcl::PointCloud<pcl::PointXYZ> perfectAndFull;
            perfectAndFull.points = objects->points;
            perfectAndFull += tempPerfectPlane;

            if (i == 0)
            {
                perfectPlanesCloud.points = perfectAndFull.points;
            }
            else
            {
                // tempPerfectPlane.points = objects->points;
                perfectPlanesCloud += perfectAndFull;
            }            
            // pcl::PointCloud<pcl::PointXYZ> tempPerfectPlane;
            // if (i == 0)
            // {
            //     perfectPlanesCloud.points = perfectPlane->points;
            // }
            // else
            // {
            //     tempPerfectPlane.points = perfectPlane->points;
            //     perfectPlanesCloud += tempPerfectPlane;
            // }
            // //////////////////////////////////////////////////////

            // adding the scraps to  the original point cloud
            cloud1->points = scraps_cloud->points;

            PointCloud::Ptr rest_cloud_msg(new PointCloud);
            std::cerr << "??????????????????????????????????" << std::endl;
            std::cerr << "Final rest_cloud size!" << cloud1->size() << std::endl;
            rest_cloud_msg->header.stamp = cloud_in_msg->header.stamp;
            rest_cloud_msg->header.frame_id = "base_link";
            rest_cloud_msg->points = cloud1->points;
            pub6_.publish(rest_cloud_msg);

            // in case something goes wrong... 5 planes should be enough, so stop the loop
            if (i == 5)
            {
                break;
            }

            ros::Duration(_planesDelay).sleep();
        }
        // adding all the remained scraps and all the rectified planes into the same cloud
        pcl::PointCloud<pcl::PointXYZ> gt_cloud;
        gt_cloud.points = cloud1->points;
        gt_cloud += perfectPlanesCloud;

        PointCloud::Ptr gt_cloud_msg(new PointCloud);
        std::cerr << "------------------------------------" << std::endl;
        // std::cerr << "Nr_planes: " << n_planes << std::endl;
        gt_cloud_msg->header.frame_id = "base_link";
        gt_cloud_msg->header.stamp = cloud_in_msg->header.stamp;
        gt_cloud_msg->points = gt_cloud.points;
        pub3_.publish(gt_cloud_msg);
    }
    // 1.15
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub2_;
    ros::Publisher pub3_;
    ros::Publisher pub4_;
    ros::Publisher pub5_;
    ros::Publisher pub6_;
    ros::Publisher pub7_;
    ros::Publisher pub8_;
    ros::Publisher pub9_;
    int _firstTime = 1;
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
