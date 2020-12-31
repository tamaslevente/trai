#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>

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
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/cloud_viewer.h>

#include "matplotlibcpp.h"

#include <stop_watch.h>

using namespace cv;
using namespace std;
namespace plt = matplotlibcpp;

class Cloud2Depth
{
private:
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    image_transport::Subscriber sub2_;
    image_transport::Publisher pub_;
    // calibration parameters 
    double K[9] = {385.8655956930966, 0.0, 342.3593021849471,
                   0.0, 387.13463636528166, 233.38372018194542,
                   0.0, 0.0, 1.0};

    // EEPROM parameters
    // double K[9] = {386.804, 0.0, 341.675,
    //                0.0, 384, 238.973,
    //                0.0, 0.0, 1.0};

    double centerX_ = K[2];
    double centerY_ = K[5];
    double focalX_ = K[0];
    double focalY_ = K[4];
    int height_ = 480;
    int width_ = 640;
    int pixel_pos_x, pixel_pos_y;
    float z, u, v;
    Mat cv_image;
    bool depthReceived = false;
    bool cloudReceived = false;
    const PointCloud::ConstPtr global_cloud_in;
    // const sensor_msgs::ImageConstPtr *global_img_msg;
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points;
    sensor_msgs::Image cam;

public:
    // typedef pcl::PointXYZ Point;
    // typedef pcl::PointCloud<Point> PointCloud;

    /*
   * constructor
   * setup which topics are passed out (advertise) and to which topics are listend (subscribe)
   */
    Cloud2Depth() : it_(nh_)
    {
        // global_cloud_in_msg = nullptr;
        // global_img_msg = nullptr;
        pub_ = it_.advertise("/aditof_roscpp/aditof_cloud_2_depth", 1);

        // cv_image = Mat(height_, width_, CV_32FC1, Scalar(std::numeric_limits<float>::max()));

        sub_ = nh_.subscribe("/aditof_roscpp/aditof_pcloud_rect", 5, &Cloud2Depth::cloudCallback, this);
        // sub2_ = it_.subscribe("/aditof_depth_32FC1_format", 5, &Cloud2Depth::depthCallback, this);
        sub2_ = it_.subscribe("/aditof_roscpp/image_rect", 5, &Cloud2Depth::depthCallback, this);

        double dist_thr;
        int max_its;

        // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
        ros::NodeHandle private_nh("~");
    }

    ~Cloud2Depth() {}

    void depthCompare() //const sensor_msgs::ImageConstPtr *img_msg)
    {
        cloudReceived = false;
        depthReceived = false;

        double nr = (double)std::numeric_limits<float>::max();
        cv_image = Mat(height_, width_, CV_32FC1, 0.0); //Scalar(std::numeric_limits<float>::max()));
        // cv::Mat depthMat(height_width_CV32FC1,cv_ptr.px.image->data)
        Mat orig_image = cv_ptr->image;
        // orig_image.convertTo(orig_image, CV_32FC1);

        // PrecisionStopWatch sw;

        // cv_ptr.px.image.at<float>(1,1)

        int numberOfZeroDepthPoints = 0;
        int suspectedProblemX = 0;
        int suspectedProblemY = 0;
        int suspectedBigProblemX = 0;
        int suspectedBigProblemY = 0;

        for (int i = 0; i < points.size(); i++)
        {
            if (points[i].z == points[i].z)
            {
                if (points[i].z != 0.0) //|| (points[i].x != 0.0 && points[i].y != 0.0))
                {
                    z = points[i].z * 1000.0;
                    u = (points[i].x * 1000.0 * focalX_) / z;
                    v = (points[i].y * 1000.0 * focalY_) / z;
                    pixel_pos_x = (int)(u + centerX_);
                    pixel_pos_y = (int)(v + centerY_);

                    if (pixel_pos_x > (width_ - 1))
                    {
                        pixel_pos_x = width_ - 1;
                        suspectedBigProblemX++;
                    }
                    else if (pixel_pos_x < 0)
                    {
                        pixel_pos_x = -pixel_pos_x;
                        suspectedProblemX++;
                    }
                    if (pixel_pos_y > (height_ - 1))
                    {
                        pixel_pos_y = height_ - 1;
                        suspectedBigProblemY++;
                    }
                    else if (pixel_pos_y < 0)
                    {
                        pixel_pos_y = -pixel_pos_y;
                        suspectedProblemY++;
                    }
                }
                else
                {
                    pixel_pos_x = 0;
                    pixel_pos_y = 0;
                    z = 0.0;
                    numberOfZeroDepthPoints++;
                }
                // modifiedZ.at(i) = z;

                cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z; 
            }
        }

        printf("%d\n", numberOfZeroDepthPoints);

        int indexVector = 0;
        int vectorSize = width_ * height_;
        // std::vector<float> originalZ(vectorSize), modifiedZ(vectorSize), index(vectorSize);
        // for (int i = 0; i < height_; i++)
        // {
        //     for (int j = 0; j < width_; j++)
        //     {
        //         // printf("i: %d, j: %d\n", i, j);
        //         indexVector = width_ * i + j;
        //         modifiedZ.at(indexVector) = cv_image.at<uint16_t>(j, i);
        //         originalZ.at(indexVector) = orig_image.at<uint16_t>(j, i);
        //         index.at(indexVector) = indexVector;
        //     }
        // }
        // // originalZ.at(4320) = 0;
        // plt::plot(index, originalZ, "b-", index, modifiedZ, "r--");
        // plt::show();

        cv_image.convertTo(cv_image, CV_16UC1);

        std::vector<float> originalZ(vectorSize), modifiedZ(vectorSize), index(vectorSize);
        for (int i = 0; i < height_; i++)
        {
            for (int j = 0; j < width_; j++)
            {
                indexVector = width_ * i + j;
                // printf("i: %d, j: %d\n", i, j);
                // printf("cv %d;\n",cv_image.at<uint16_t>(i, j));
                // printf("orig %d;\n",orig_image.at<uint16_t>(i, j));
                modifiedZ.at(indexVector) = cv_image.at<uint16_t>(i, j);
                originalZ.at(indexVector) = orig_image.at<uint16_t>(i, j);
                index.at(indexVector) = indexVector;
            }
        }

        // Mat diff_image = cv_image-orig_image;
        // plt::plot(index, originalZ, "b-", index, modifiedZ, "r--");
        // plt::show();

        int numberOfZeroPixels = 0;
        for (int i = 0; i < width_; i++)
        {
            for (int j = 0; j < height_; j++)
            {
                uint pixel = cv_image.at<uint16_t>(j, i);
                // printf("%d;\n",pixel);
                if (pixel == 0.0)
                {
                    numberOfZeroPixels++;
                }
            }
        }
        printf("%d\n", numberOfZeroPixels);

        // // imshow("Display depth from point cloud", cv_image);

        sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
        // // output_image->header = info.header;
        // // output_image->header.stamp = info.header.stamp = t;
        // // cv::waitKey(3);
        pub_.publish(output_image);
    }

    void depthCallback(const sensor_msgs::ImageConstPtr &img_msg)
    {
        cam = *img_msg;
        try
        {
            // cv_ptr = cv_bridge::toCvShare(img_msg);
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // global_img_msg = &img_msg;
        depthReceived = true;
        if (depthReceived == true && cloudReceived == true)
        {
            depthCompare();
        }
        // waitKey(3);
    }

    void cloudCallback(const PointCloud::ConstPtr &cloud_in_msg)
    {
        if (!cloud_in_msg || cloud_in_msg->size() <= 0)
        {
            ROS_WARN("got empty or invalid pointcloud --> ignoring");
            return;
        }
        points = cloud_in_msg->points;
        // &global_cloud_in = &cloud_in_msg;
        cloudReceived = true;
        if (depthReceived == true && cloudReceived == true)
        {
            depthCompare();
        }
        // waitKey(3);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_2_depth_node");
    // ros::AsyncSpinner spinner(0);
    // spinner.start();

    Cloud2Depth c2d;

    // ros::waitForShutdown();
    ros::spin();
    return 0;
}
