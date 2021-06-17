
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>

using namespace std;
using namespace cv;

cv::Mat createDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double K[], int height, int width)
{
    int pixel_pos_x, pixel_pos_y;
    double z, u, v;
    cv::Mat cv_image;
    std::vector<Point2d> imagePoints;
    double fx = K[0];
    double fy = K[4];
    double x0 = K[2];
    double y0 = K[5];
    cv::Mat output = cv::Mat::zeros(height, width, CV_16UC1);
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
            z = cloud->points[i].z * 1000.0;
            u = (cloud->points[i].x * 1000.0 * fx) / z;
            v = (cloud->points[i].y * 1000.0 * fy) / z;
            pixel_pos_x = (int)(u + x0);
            pixel_pos_y = (int)(v + y0);

            if (pixel_pos_x < 0)
            {
                pixel_pos_x = -pixel_pos_x;
            }
            if (pixel_pos_x > (width - 1))
            {
                pixel_pos_x = width - 1;
            }

            if (pixel_pos_y < 0)
            {
                pixel_pos_y = -pixel_pos_y;
            }
            if (pixel_pos_y > (height - 1))
            {
                pixel_pos_y = height - 1;
            }
            output.at<uint16_t>(pixel_pos_y, pixel_pos_x) = z;
        }
    }
    return output;
}

int main(int argc, char **argv)
{
    std::string gtdir = argv[1];
    std::string preddir = argv[2];
    std::string gtending = argv[3];
    std::string predending = argv[4];
    char file_gt[200];
    char file_pred[200];
    int start = atoi(argv[5]);
    int end = atoi(argv[6]);
    int step = atoi(argv[7]);
    std::string camera_type = argv[8];
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
    int height = atoi(argv[9]);
    int width = atoi(argv[10]);
    int count = start;
    int cnt = 0;
    float avg_missing = 0;
    float avg_extra = 0;
    float avg_difference = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudgt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpred(new pcl::PointCloud<pcl::PointXYZ>);
    while (count < end)
    {

        sprintf(file_gt, "%s%05d%s", gtdir.c_str(), count, gtending.c_str());
        sprintf(file_pred, "%s%05d%s", preddir.c_str(), count, predending.c_str());

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_gt, *cloudgt) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file \n");
            return (-1);
        }
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_pred, *cloudpred) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file \n");
            return (-1);
        }
        std::cout << count << std::endl;
        cv::Mat mat_depthgt = createDepth(cloudgt, K, height, width);
        cv::Mat mat_depthpred = createDepth(cloudpred, K, height, width);

        float missing = 0;
        float extra = 0;
        float difference = 0;
        for (int x = 0; x < mat_depthgt.rows; x++)
        {
            for (int y = 0; y < mat_depthgt.cols; y++)
            {
                if (mat_depthgt.at<uint16_t>(x, y) != mat_depthpred.at<uint16_t>(x, y))
                {
                    if (mat_depthgt.at<uint16_t>(x, y) == 0)
                    {
                        extra++;
                    }
                    else
                    {
                        if (mat_depthpred.at<uint16_t>(x, y) == 0)
                        {
                            missing++;
                        }
                        else
                        {
                            difference += abs(mat_depthgt.at<uint16_t>(x, y) - mat_depthpred.at<uint16_t>(x, y));
                        }
                    }
                }
            }
        }
        avg_missing += missing;
        avg_extra += extra;
        avg_difference += difference / (mat_depthgt.rows * mat_depthgt.cols - extra - missing);
        count = count + step;
        cnt++;
    }
    avg_missing /= cnt;
    avg_extra /= cnt;
    avg_difference /= cnt;
    printf("Avg. missing: %f\nAvg. extra: %f\nAvg. difference: %f\n", avg_missing, avg_extra, avg_difference);

    return (0);
}
