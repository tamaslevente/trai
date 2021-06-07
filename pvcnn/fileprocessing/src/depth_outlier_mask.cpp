#include <ctime>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr createPCD(cv::Mat depth, double K[])
{
    double fx = K[0];
    double fy = K[4];
    double x0 = K[2];
    double y0 = K[5];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p;
    for (int i = 0; i < depth.rows; i++)
    {
        for (int j = 0; j < depth.cols; j++)
        {
            int index = i * depth.cols + j;
            float d = depth.at<uint16_t>(i, j) / 1000.0;

            if (d == 0.0)
            {
                continue;
            }
            float x_over_z = (j - x0) / fx;
            float y_over_z = (i - y0) / fy;
            p.z = d;
            p.x = x_over_z * p.z;
            p.y = y_over_z * p.z;

            cloud->points.push_back(p);
        }
    }
    cloud->width = cloud->points.size();
    std::cout << "Number of points before:" << cloud->width << std::endl;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->is_dense = false;
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr sorFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, int stddevMulThresh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*cloud_filtered);
    std::cout << "Number of points after:" << cloud_filtered->width << std::endl;
    return cloud_filtered;
}

cv::Mat createDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double K[], int R, int C)
{
    double fx = K[0];
    double fy = K[4];
    double x0 = K[2];
    double y0 = K[5];
    cv::Mat output = cv::Mat::zeros(R, C, CV_8UC1);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        bool nan = false;
        if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
            nan = true;
        if (isinf(cloud->points[i].x) || isinf(cloud->points[i].y) || isinf(cloud->points[i].z))
            nan = true;
        if (cloud->points[i].z <= (fy / 1000.0))
            nan = true;
        if (!nan)
        {
            double z = cloud->points[i].z * 1000.0;
            double u = (cloud->points[i].x * 1000.0 * fx) / z;
            double v = (cloud->points[i].y * 1000.0 * fy) / z;
            int pixel_pos_x = (int)(u + x0);
            int pixel_pos_y = (int)(v + y0);

            if (pixel_pos_x < 0)
            {
                pixel_pos_x = -pixel_pos_x;
            }
            if (pixel_pos_x > (C - 1))
            {
                pixel_pos_x = C - 1;
            }

            if (pixel_pos_y < 0)
            {
                pixel_pos_y = -pixel_pos_y;
            }
            if (pixel_pos_y > (R - 1))
            {
                pixel_pos_y = R - 1;
            }
            output.at<uint8_t>(pixel_pos_y, pixel_pos_x) = 255;
        }
    }
    return output;
}

int main(int argc, char **argv)
{
    std::string camera_type = argv[4];
    std::string camera_type_pico = "pico";
    std::string camera_type_nyu = "nyu";
    std::string camera_type_kitti = "kitti";
    double K[9] = {582.62448167737955, 0.0, 313.04475870804731, 0.0, 582.69103270988637, 238.44389626620386, 0.0, 0.0, 1.0}; // nyu_v2_dataset
    if (camera_type.compare(camera_type_pico) == 0)
    {

        K[0] = 460.58518931365654;
        K[2] = 334.0805877590529;
        K[4] = 460.2679961517268;
        K[5] = 169.80766383231037; // pico zense
    }
    if (camera_type.compare(camera_type_nyu) == 0)
    {

        K[0] = 582.62448167737955;
        K[2] = 313.04475870804731;
        K[4] = 582.69103270988637;
        K[5] = 238.44389626620386; // nyu v2
    }
    if (camera_type.compare(camera_type_kitti) == 0)
    {

        K[0] = 721.5377;
        K[2] = 609.5593;
        K[4] = 721.5377;
        K[5] = 149.854; // kitti - average
    }
    int meanK = atoi(argv[5]);
    int stddevMulThresh = atof(argv[6]);
    char file_depthin[200];
    char file_depthmask[200];
    std::string ddir = argv[1];
    std::string mdir = argv[2];
    std::string filename = argv[3];

    const clock_t begin_time = clock();
    sprintf(file_depthin, "%s%s", ddir.c_str(), filename.c_str());
    filename = filename.substr(0, filename.size() - 9);
    sprintf(file_depthmask, "%s%s_mask.png", mdir.c_str(), filename.c_str());
    printf("Processing file - %s", file_depthin);
    cv::Mat depth = cv::imread(file_depthin, CV_LOAD_IMAGE_UNCHANGED);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = createPCD(depth, K);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = sorFilter(cloud, meanK, stddevMulThresh);
    cv::Mat output = createDepth(cloud_filtered, K, depth.rows, depth.cols);
    imwrite(file_depthmask, output);
    std::cout << "time:" << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;

    return (0);
}