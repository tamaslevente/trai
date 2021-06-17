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
    std::string idir = argv[1];
    std::string odir = argv[2];
    std::string filename = argv[3];
    char file_in[200];
    sprintf(file_in, "%s%s", idir.c_str(), filename.c_str());
    filename = filename.substr(0, filename.size() - 3);
    char file_out[200];
    sprintf(file_out, "%s%spcd", odir.c_str(), filename.c_str());

    ifstream normal_file(file_in);
    //Check that the input file has being successfully opened
    if (!(normal_file.is_open()))
    {
        cout << "[x] Cannot open normal file!" << endl;
    }

    string content = "";
    string coords = "";
    string normals = "";
    int i = 0;
    while (getline(normal_file, content))
    {
        boost::split(result, content, [](char c) { return c == ' '; });
        p.x = stof(result[0]);
        p.y = stof(result[1]);
        p.z = stof(result[2]);
        p.normal_x = stof(result[3]);
        p.normal_y = stof(result[4]);
        p.normal_z = stof(result[5]);
        cloud.points.push_back(p);
        i++;
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
