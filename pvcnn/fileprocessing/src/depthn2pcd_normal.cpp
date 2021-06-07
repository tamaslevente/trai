#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
    vector<string> norms;
    pcl::PointCloud<pcl::PointNormal> cloud;
    pcl::PointNormal p;

    std::string ddir = argv[1];
    std::string ndir = argv[2];
    std::string odir = argv[3];
    std::string filename = argv[4];
    std::string nfilename = argv[5];
    char file_depth[200];
    sprintf(file_depth, "%s%s", ddir.c_str(), filename.c_str());
    char file_normal[200];
    sprintf(file_normal, "%s%s", ndir.c_str(), nfilename.c_str());
    std::cout<<file_normal<<std::endl;
    filename = filename.substr(0, filename.size() - 3);
    char file_out[200];
    sprintf(file_out, "%s%spcd", odir.c_str(), filename.c_str());

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

    cv::Mat mat_depth = cv::imread(file_depth, CV_LOAD_IMAGE_UNCHANGED);
    ifstream normal_file(file_normal);
    //Check that the input file has being successfully opened
    if (!(normal_file.is_open()))
    {
        cout << "[x] Cannot open normal file!" << endl;
    }

    string normals = "";
    int i = 0;
    int j = 0;
    while (getline(normal_file, normals))
    {
        double d = (double)mat_depth.at<uint16_t>(i, j) / 1000.0;

        if (d != 0.0)
        {
            boost::split(norms, normals, [](char c) { return c == ' '; });
            float x_over_z = (j - x0) / fx;
            float y_over_z = (i - y0) / fy;
            p.z = d;
            p.x = x_over_z * p.z;
            p.y = y_over_z * p.z;
            p.normal_x = stof(norms[0]);
            p.normal_y = stof(norms[1]);
            p.normal_z = stof(norms[2]);
            cloud.points.push_back(p);
        }

        j++;
        if (j >= mat_depth.cols)
        {
            j = 0;
            i++;
        }
    }

    cloud.width = cloud.points.size();
    cout << "Number of points:" << cloud.width << endl;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.is_dense = false;
    normal_file.close();
    pcl::io::savePCDFile(file_out, cloud, true);
    cout << "[*] Conversion finished!" << endl;
    return 0;
}
