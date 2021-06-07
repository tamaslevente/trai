#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;

int main(int argc, char *argv[])
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
    std::string idir = argv[1];
    std::string odir = argv[2];
    std::string filename = argv[3];
    char file_in[200];
    sprintf(file_in, "%s%s", idir.c_str(), filename.c_str());
    printf("Processing: %s\n",filename.c_str());
    filename = filename.substr(0, filename.size() - 4);
    char file_out[200];
    sprintf(file_out, "%s%s.pcd", odir.c_str(), filename.c_str());
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_in, *cloud0) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }
    // filename = filename.substr(0, filename.size() - 4);
    std::cout << "PointCloud has: " << cloud0->size() << " data points." << std::endl; //*
    bool nan = false;
    for (int i = 0; i < cloud0->size(); i++)
    {
        p.x = cloud0->points[i].x;
        p.y = cloud0->points[i].y;
        p.z = cloud0->points[i].z;
        if (p.x != p.x || p.y != p.y || p.z!= p.z || std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
            nan = true;
        if (!nan)
            cloud.points.push_back(p);
        nan = false;
    }

    cloud.width = cloud.points.size();
    cout << "Number of points after sweeping:" << cloud.width << endl;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.is_dense = false;
    pcl::io::savePCDFile(file_out, cloud, true);

    cout << "[*] Conversion finished!" << endl;
    return 0;
}
