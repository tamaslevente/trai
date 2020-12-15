// Include opencv2
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <pcl/range_image/range_image_planar.h>

// #include "matplotlibcpp.h"

using namespace cv;
// namespace plt = matplotlibcpp;

using namespace std;

int main()
{

    // calibration parameters
    // double K[9] = {385.8655956930966, 0.0, 342.3593021849471,
    //                0.0, 387.13463636528166, 233.38372018194542,
    //                0.0, 0.0, 1.0};

    // EEPROM parameters
    double K[9] = {460.585, 0.0, 334.080,
                   0.0, 460.267, 169.807,
                   0.0, 0.0, 1.0};

    double centerX_ = K[2];
    double centerY_ = K[5];
    double focalX_ = K[0];
    double focalY_ = K[4];
    int height_ = 360;
    int width_ = 640;
    int pixel_pos_x, pixel_pos_y;
    float z, u, v;
    cv::Mat cv_image;
    std::vector<Point2d> imagePoints;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string filename = "";
    std::cin >> filename;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }
    std::cout << "PointCloud has: " << cloud->size() << " data points." << std::endl;
    //cv_image = Mat(height_, width_, CV_32FC1, 0.0); //Scalar(std::numeric_limits<float>::max()));
    cv::Mat output = cv::Mat::zeros(height_, width_, CV_8UC3);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        uint32_t rgb = cloud->points[i].rgb;
        
        uint8_t r = cloud->points[i].r;
        uint8_t g = cloud->points[i].g;
        uint8_t b = cloud->points[i].b;
        //std::cout<<"rgb="<<(int)r<<", "<<(int)g<<", "<<(int)b<<std::endl;

        z = cloud->points[i].z * 1000.0;
        u = (cloud->points[i].x * 1000.0 * focalX_) / z;
        v = (cloud->points[i].y * 1000.0 * focalY_) / z;

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

        output.at<Vec3b>(pixel_pos_y, pixel_pos_x)[0] = b;
        output.at<Vec3b>(pixel_pos_y, pixel_pos_x)[1] = g;
        output.at<Vec3b>(pixel_pos_y, pixel_pos_x)[2] = r;
    }

    //output.convertTo(output, CV_16UC3);

    // imshow("Display depth from point cloud", cv_image);
    waitKey(3);
    filename = filename.substr(0, filename.size() - 3);
    imwrite(filename + "png", output);
    //sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
    //pub_.publish(output_image);
}
