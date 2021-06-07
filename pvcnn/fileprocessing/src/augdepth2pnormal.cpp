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

char directory[100] = "/media/rambo/ssd2/Szilard/file_repository/nyu_v2_augmented/";
int cnt = 0;
int counter = 0;
int counter2 = 8694;
int max_nr = 1449;

// double K[9] = {460.58518931365654, 0.0, 334.0805877590529, 0.0, 460.2679961517268, 169.80766383231037, 0.0, 0.0, 1.0}; // pico zense
// double K[9] = {582.62448167737955, 0.0, 313.04475870804731, 0.0, 582.69103270988637, 238.44389626620386, 0.0, 0.0, 1.0}; // nyu_v2_dataset
double K[9] = {721.5377, 0.0, 609.5593, 0.0, 721.5377, 149.854, 0.0, 0.0, 1.0}; // kitti dataset - average
double fx = K[0];
double fy = K[4];
double cx0 = K[2];
double cy0 = K[5];

void depth_normals_to_pcdnormals()
{
    char file_d3in[100];
    char file_d3inflr[100];
    char file_d3infud[100];
    char file_normal[100];
    char file_d3out1[100];
    char file_d3out2[100];
    char file_d3out3[100];
    char file_depthout1[100];
    char file_depthout2[100];
    char file_depthout3[100];
    vector<string> norms;
    pcl::PointCloud<pcl::PointNormal> cloud1;
    pcl::PointNormal p1;
    pcl::PointCloud<pcl::PointNormal> cloud2;
    pcl::PointNormal p2;
    pcl::PointCloud<pcl::PointNormal> cloud3;
    pcl::PointNormal p3;
    printf("Converting file: %snormals/%05d.normals\n", directory, cnt);
    sprintf(file_d3in, "%snoisydepth3/%05d_5.png", directory, counter);
    sprintf(file_d3inflr, "%snoisydepth3/%05d_5.png", directory, counter + 1);
    sprintf(file_d3infud, "%snoisydepth3/%05d_5.png", directory, counter + 2);
    sprintf(file_normal, "%snormals/%05d.normals", directory, cnt);
    sprintf(file_d3out1, "%sadepth3/%05d_depth3.png", directory, counter2);
    sprintf(file_d3out2, "%sadepth3/%05d_depth3.png", directory, counter2 + 1);
    sprintf(file_d3out3, "%sadepth3/%05d_depth3.png", directory, counter2 + 2);
    sprintf(file_depthout1, "%sadepth/%05d_depth.png", directory, counter2);
    sprintf(file_depthout2, "%sadepth/%05d_depth.png", directory, counter2 + 1);
    sprintf(file_depthout3, "%sadepth/%05d_depth.png", directory, counter2 + 2);
    cv::Mat depth31 = cv::imread(file_d3in, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat depth32 = cv::imread(file_d3inflr, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat depth33 = cv::imread(file_d3infud, CV_LOAD_IMAGE_UNCHANGED);
    vector<cv::Mat> channels1(3);
    cv::split(depth31, channels1);
    cv::Mat depth1 = channels1[1];
    vector<cv::Mat> channels2(3);
    cv::split(depth32, channels2);
    cv::Mat depth2 = channels2[1];
    vector<cv::Mat> channels3(3);
    cv::split(depth33, channels3);
    cv::Mat depth3 = channels3[1];
    cv::imwrite(file_depthout1, depth1);
    cv::imwrite(file_d3out1, depth31);
    cv::imwrite(file_depthout2, depth2);
    cv::imwrite(file_d3out2, depth32);
    cv::imwrite(file_depthout3, depth3);
    cv::imwrite(file_d3out3, depth33);

    ifstream normal_file(file_normal);
    //Check that the input file has being successfully opened
    if (!(normal_file.is_open()))
    {
        cout << "[x] Cannot open normal file!" << endl;
    }

    string normals = "";
    int i1 = 0;
    int j1 = 0;
    int i2 = 0;
    int j2 = depth2.cols - 1;
    int i3 = depth3.rows - 1;
    int j3 = 0;
    while (getline(normal_file, normals))
    {
        double d1 = (double)depth1.at<uint16_t>(i1, j1) / 1000.0;

        if (d1 != 0.0)
        {
            boost::split(norms, normals, [](char c) { return c == ' '; });
            float x_over_z = (j1 - cx0) / fx;
            float y_over_z = (i1 - cy0) / fy;
            p1.z = d1;
            p1.x = x_over_z * p1.z;
            p1.y = y_over_z * p1.z;
            p1.normal_x = stof(norms[0]);
            p1.normal_y = stof(norms[1]);
            p1.normal_z = stof(norms[2]);
            cloud1.points.push_back(p1);

            x_over_z = (j2 - cx0) / fx;
            y_over_z = (i2 - cy0) / fy;
            p2.z = d1;
            p2.x = x_over_z * p2.z;
            p2.y = y_over_z * p2.z;
            p2.normal_x = -stof(norms[0]);
            p2.normal_y = stof(norms[1]);
            p2.normal_z = stof(norms[2]);
            cloud2.points.push_back(p2);

            x_over_z = (j3 - cx0) / fx;
            y_over_z = (i3 - cy0) / fy;
            p3.z = d1;
            p3.x = x_over_z * p3.z;
            p3.y = y_over_z * p3.z;
            p3.normal_x = stof(norms[0]);
            p3.normal_y = -stof(norms[1]);
            p3.normal_z = stof(norms[2]);
            cloud3.points.push_back(p3);
        }

        j1++;
        if (j1 >= depth1.cols)
        {
            j1 = 0;
            i1++;
        }
        j2--;
        if (j2 < 0)
        {
            j2 = depth2.cols - 1;
            i2++;
        }
        j3++;
        if (j3 >= depth3.cols)
        {
            j3 = 0;
            i3--;
        }
    }
    normal_file.close();

    cloud1.width = cloud1.points.size();
    cout << "Number of points:" << cloud1.width << endl;
    cloud1.height = 1;
    cloud1.points.resize(cloud1.width * cloud1.height);
    cloud1.is_dense = false;
    char file_pcd1[100];
    sprintf(file_pcd1, "%spnormals/%05d_pclouds_normals.pcd", directory, counter2);
    pcl::io::savePCDFile(file_pcd1, cloud1, true);

    cloud2.width = cloud2.points.size();
    cout << "Number of points:" << cloud2.width << endl;
    cloud2.height = 1;
    cloud2.points.resize(cloud2.width * cloud2.height);
    cloud2.is_dense = false;
    char file_pcd2[100];
    sprintf(file_pcd2, "%spnormals/%05d_pclouds_normals.pcd", directory, counter2 + 1);
    pcl::io::savePCDFile(file_pcd2, cloud2, true);

    cloud3.width = cloud3.points.size();
    cout << "Number of points:" << cloud3.width << endl;
    cloud3.height = 1;
    cloud3.points.resize(cloud3.width * cloud3.height);
    cloud3.is_dense = false;
    char file_pcd3[100];
    sprintf(file_pcd3, "%spnormals/%05d_pclouds_normals.pcd", directory, counter2 + 2);
    pcl::io::savePCDFile(file_pcd3, cloud3, true);

    cout << "[*] Conversion finished!" << endl;
    counter = counter + 3;
    counter2 = counter2 + 3;
    cnt ++;
}

int main(int argc, char *argv[])
{
    while (cnt < max_nr)
    {
        depth_normals_to_pcdnormals();
    }
    return 0;
}
