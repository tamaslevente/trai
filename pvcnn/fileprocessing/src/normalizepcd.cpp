#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    std::string idir = argv[1];
    std::string odir = argv[2];
    std::string filename = argv[3];
    char file_in[200];
    sprintf(file_in, "%s%s", idir.c_str(), filename.c_str());
    char file_out[200];
    sprintf(file_out, "%s%s", odir.c_str(), filename.c_str());

    if (pcl::io::loadPCDFile<pcl::PointNormal>(file_in, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }
    std::cout << "PointCloud has: " << cloud->size() << " data points." << std::endl;
    double maxx=-100.0,maxy=-100.0,maxz=-100.0,minx=100.0,miny=100.0,minz=100.0;
    for (int i = 0; i < cloud->points.size(); i++){
        if(maxx<cloud->points[i].x) maxx=cloud->points[i].x;
        if(maxy<cloud->points[i].y) maxy=cloud->points[i].y;
        if(maxz<cloud->points[i].z) maxz=cloud->points[i].z;
        if(minx>cloud->points[i].x) minx=cloud->points[i].x;
        if(miny>cloud->points[i].y) miny=cloud->points[i].y;
        if(minz>cloud->points[i].z) minz=cloud->points[i].z;
    }
    double divx=1.0, divy=1.0, divz=5.0, maxsize=1.0, zcenter=1.5, div=1.0;
    // if(abs(maxx)>abs(minx)) divx=maxsize/abs(maxx);
    // else divx=maxsize/abs(minx);
    // if(abs(maxy)>abs(miny)) divy=maxsize/abs(maxy);
    // else divy=maxsize/abs(miny);
    // if(abs(maxz)>abs(minz)) divz=maxsize/abs(maxz);
    // else divz=maxsize/abs(minz);
    divx=maxsize/abs(maxx-minx);
    divy=maxsize/abs(maxy-miny);
    divz=maxsize/abs(maxz-minz);
    if(divx<divy && divx<divz)  div=divx;
    if(divy<divx && divy<divz) div=divy;
    if(divz<divy && divz<divx) div=divz;
    // if(divx>divy && divx>divz && divx>=1.0) div=divx;
    // if(divy>divx && divy>divz && divy>=1.0) div=divy;
    // if(divz>divy && divz>divx && divz>=1.0) div=divz;
    
    double shiftx=(maxx+minx)/2*div, shifty=(maxy+miny)/2*div, shiftz=(maxz+minz)/2*div;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x=cloud->points[i].x*div-shiftx;
        cloud->points[i].y=cloud->points[i].y*div-shifty;
        cloud->points[i].z=cloud->points[i].z*div-shiftz+zcenter;
    }
    pcl::io::savePCDFile(file_out, *cloud, true);
}
