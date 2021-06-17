#include <iostream>
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int main(int argc, char *argv[])
{

    vector<string> result;
    vector<string> norms;
    pcl::PointCloud<pcl::PointNormal> cloud;
    pcl::PointNormal p;
    std::string xyzdir = argv[1];
    std::string ndir = argv[2];
    std::string odir = argv[3];
    std::string filename = argv[4];
    std::string nfilename = argv[5];
    char file_xyz[200];
    sprintf(file_xyz, "%s%s", xyzdir.c_str(), filename.c_str());
    char file_normal[200];
    sprintf(file_normal, "%s%s", ndir.c_str(), nfilename.c_str());
    filename = filename.substr(0, filename.size() - 3);
    char file_out[200];
    sprintf(file_out, "%s%spcd", odir.c_str(), filename.c_str());

    ifstream xyz_file(file_xyz);
    //Check that the input file has being successfully opened
    if (!(xyz_file.is_open()))
    {
        cout << "[x] Cannot open xyz file!" << endl;
    }

    ifstream normal_file(file_normal);
    //Check that the input file has being successfully opened
    if (!(normal_file.is_open()))
    {
        cout << "[x] Cannot open normal file!" << endl;
    }

    string content = "";
    string normals = "";
    int i = 0;
    while (getline(normal_file, normals))
    {
        getline(xyz_file, content);
        //getline(normal_file,normals);
        boost::split(result, content, [](char c) { return c == ' '; });
        boost::split(norms, normals, [](char c) { return c == ' '; });
        p.x = stof(result[0]);
        p.y = stof(result[1]);
        p.z = stof(result[2]);
        p.normal_x = stof(norms[0]);
        p.normal_y = stof(norms[1]);
        p.normal_z = stof(norms[2]);
        cloud.points.push_back(p);
        i++;
    }

    cloud.width = cloud.points.size();
    cout << "Number of points:" << cloud.width << endl;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.is_dense = false;
    xyz_file.close();
    normal_file.close();
    pcl::io::savePCDFile(file_out, cloud, true);
    cout << "[*] Conversion finished!" << endl;
    return 0;
}
