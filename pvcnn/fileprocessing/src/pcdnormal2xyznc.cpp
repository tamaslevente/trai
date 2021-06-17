#include <iostream>
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;

int main(int argc, char *argv[])
{
    std::string idir = argv[1];
    std::string odir = argv[2];
    std::string filename = argv[3];
    char file_in[200];
    sprintf(file_in, "%s%s", idir.c_str(), filename.c_str());
    filename = filename.substr(0, filename.size() - 4);
    char file_normal[200];
    sprintf(file_normal, "%s%s.normals", odir.c_str(), filename.c_str());
    char file_xyz[200];
    sprintf(file_xyz, "%s%s.xyz", odir.c_str(), filename.c_str());
    char file_curv[200];
    sprintf(file_curv, "%s%s.curv", odir.c_str(), filename.c_str());

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    
    // printf("Processing: %s\n", filename);
    if (pcl::io::loadPCDFile<pcl::PointNormal>(file_in, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }
    std::cout << "PointCloud has: " << cloud->size() << " data points." << std::endl;
    
    ofstream normal_file, xyz_file, curv_file;
    xyz_file.open (file_xyz);
    normal_file.open (file_normal);
    curv_file.open (file_curv);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        xyz_file << cloud->points[i].x << " " << cloud->points[i].y <<" "<< cloud->points[i].z <<"\n";
        normal_file << cloud->points[i].normal_x << " " << cloud->points[i].normal_y <<" "<< cloud->points[i].normal_z <<"\n";
        curv_file << cloud->points[i].curvature <<"\n";
    }
    xyz_file.close();
    normal_file.close();
    curv_file.close();
    
    cout << "[*] Conversion finished!" << endl;
    return 0;
}
