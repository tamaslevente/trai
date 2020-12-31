#include <boost/filesystem.hpp>

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
#include <pcl/io/pcd_io.h>

#include <stop_watch.h>

using namespace cv;
using namespace std;
namespace fs = boost::filesystem;
// namespace plt = matplotlibcpp;

class Cloud2Depth
{
public:
    // typedef pcl::PointXYZRGB Point;
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    /*
   * constructor
   * setup which topics are passed out (advertise) and to which topics are listend (subscribe)
   */
    Cloud2Depth() : it_(nh_)
    {
        pub_ = it_.advertise("/aditof_roscpp/aditof_cloud_2_depth_static", 1);

        // sub_ = nh_.subscribe("/cloud_pcd", 1, &Cloud2Depth::cloudCallback, this);

        double dist_thr;
        int max_its;

        // "~" means, that the node hand is opened within the private namespace (to get the "own" parameters)
        ros::NodeHandle private_nh("~");
        // printf("%s",param);
        if (private_nh.getParam("pcd_folder", _param))
        {
            ROS_INFO("Got param: %s", _param.c_str());
            for (fs::directory_entry & entry : fs::directory_iterator(_param))
            {
                // cout << entry.path() << '\n';
                
            }
            
        }else
        {
            ROS_ERROR("Failed to get param 'pcd_folder' ");
        }
    }

    ~Cloud2Depth() {}

    void pcdProc(const PointCloud::ConstPtr &cloud_in_msg)
    {
        if (!cloud_in_msg || cloud_in_msg->size() <= 0)
        {
            ROS_WARN("got empty or invalid pointcloud --> ignoring");
            return;
        }
        cv_image = Mat(height_, width_, CV_32FC1, 0.0); //Scalar(std::numeric_limits<float>::max()));

        int minRange = 0; // this is the smallest distance at which the camera can measure depending on the mode (near, far, etc.)

        for (int i = 0; i < cloud_in_msg->points.size(); i++)
        {
            if (cloud_in_msg->points[i].z == cloud_in_msg->points[i].z)
            {
                if (cloud_in_msg->points[i].z != 0.0)
                {
                    z = cloud_in_msg->points[i].z * 1000.0;
                    u = (cloud_in_msg->points[i].x * 1000.0 * focalX_) / z;
                    v = (cloud_in_msg->points[i].y * 1000.0 * focalY_) / z;
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

        cv_image.convertTo(cv_image, CV_16UC1);

        // imshow("Display depth from point cloud", cv_image);
        waitKey(3);

        imwrite("depth_from_pcd.png", cv_image);
        sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
        pub_.publish(output_image);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    // ros::Subscriber sub_;
    image_transport::Publisher pub_;
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
    string _param;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_2_depth_node");

    Cloud2Depth c2d;

    ros::spin();

    return 0;
}
