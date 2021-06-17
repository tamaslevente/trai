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
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    // calibration parameters
    std::string camera_type = argv[6];
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
    double fx = K[0];
    double fy = K[4];
    double x0 = K[2];
    double y0 = K[5];
    int height_ = atoi(argv[4]);
    int width_ = atoi(argv[5]);
    int pixel_pos_x, pixel_pos_y;
    float z, u, v;
    cv::Mat cv_image;
    std::vector<Point2d> imagePoints;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string indir = argv[1];
    std::string outdir = argv[2];
    std::string filename = argv[3];
    std::cout << filename << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(indir+filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }
    std::cout << "PointCloud has: " << cloud->size() << " data points." << std::endl;
    cv::Mat output = cv::Mat::zeros(height_, width_, CV_8UC3);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        bool nan = false;
        if (isnan(cloud->points[i].r) || isnan(cloud->points[i].g) || isnan(cloud->points[i].b) || isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
            nan = true;
        if (isinf(cloud->points[i].r) || isinf(cloud->points[i].g) || isinf(cloud->points[i].b) || isinf(cloud->points[i].x) || isinf(cloud->points[i].y) || isinf(cloud->points[i].z))
            nan = true;
        if (cloud->points[i].z <= 20 / 1000)
            nan = true;
        if (!nan)
        {
            uint32_t rgb = cloud->points[i].rgb;
            uint8_t r = cloud->points[i].r;
            uint8_t g = cloud->points[i].g;
            uint8_t b = cloud->points[i].b;

            z = cloud->points[i].z * 1000.0;
            u = (cloud->points[i].x * 1000.0 * fx) / z;
            v = (cloud->points[i].y * 1000.0 * fy) / z;
            pixel_pos_x = (int)(u + x0);
            pixel_pos_y = (int)(v + y0);

            if (pixel_pos_x < 0)
            {
                pixel_pos_x = -pixel_pos_x;
            }
            if (pixel_pos_x > (width_ - 1))
            {
                pixel_pos_x = width_ - 1;
            }

            if (pixel_pos_y < 0)
            {
                pixel_pos_y = -pixel_pos_y;
            }
            if (pixel_pos_y > (height_ - 1))
            {
                pixel_pos_y = height_ - 1;
            }
            output.at<Vec3b>(pixel_pos_y, pixel_pos_x)[0] = b;
            output.at<Vec3b>(pixel_pos_y, pixel_pos_x)[1] = g;
            output.at<Vec3b>(pixel_pos_y, pixel_pos_x)[2] = r;
        }
    }
    waitKey(3);
    filename = filename.substr(0, filename.size() - 3);
    imwrite(outdir+filename + "png", output);
}
