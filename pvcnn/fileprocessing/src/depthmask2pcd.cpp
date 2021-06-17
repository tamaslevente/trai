#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
    std::string ddir = argv[1];
    std::string mdir = argv[2];
    std::string pcddir = argv[3];
    std::string dfilename = argv[4];
    std::string mfilename = argv[5];

    char file_depth[200];
    char file_mask[200];
    char file_pcd[200];

    sprintf(file_depth, "%s%s", ddir.c_str(), dfilename.c_str());
    std::cout<<"Processing: "<<file_depth<<std::endl;
    dfilename = dfilename.substr(0, dfilename.size() - 4);
    sprintf(file_mask, "%s%s", mdir.c_str(), mfilename.c_str());    
    sprintf(file_pcd, "%s%s.pcd", pcddir.c_str(), dfilename.c_str());

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

    // Point Clouds to hold output
    cv::Mat depth = cv::imread(file_depth, cv::IMREAD_UNCHANGED);
    cv::Mat mask = cv::imread(file_mask, cv::IMREAD_GRAYSCALE);
    
    pcl::PointCloud<pcl::PointXYZ> cloud_msg;
    pcl::PointXYZ p;
    bool nan = false;
    for (int i = 0; i < depth.rows; i++)
    {
        for (int j = 0; j < depth.cols; j++)
        {
            float d = depth.at<uint16_t>(i, j) / 1000.0;
            float m = mask.at<uint8_t>(i, j);
            if (d == 0.0 || m < 255)
            {
                continue;
            }
            float x_over_z = (j - x0) / fx;
            float y_over_z = (i - y0) / fy;
            p.z = d;
            p.x = x_over_z * p.z;
            p.y = y_over_z * p.z;
            cloud_msg.points.push_back(p);
        }
    }
    
    cloud_msg.width = cloud_msg.points.size();
    cout << "Number of points:" << cloud_msg.width << endl;
    cloud_msg.height = 1;
    cloud_msg.points.resize(cloud_msg.width * cloud_msg.height);
    cloud_msg.is_dense = false;
    
    pcl::io::savePCDFile(file_pcd, cloud_msg, true);
    cout << "[*] Conversion finished!" << endl;

    return 0;
}
