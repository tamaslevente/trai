#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ctime>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    std::string idir = argv[1];
    std::string odir = argv[2];
    std::string filename = argv[3];
    int MeanK=atoi(argv[4]);
    float stddevMultiTresh = atof(argv[5]);
    char file_in[200];
    sprintf(file_in, "%s%s", idir.c_str(), filename.c_str());
    filename = filename.substr(0, filename.size() - 4);
    char file_out[200];
    sprintf(file_out, "%s%s_sor.pcd", odir.c_str(), filename.c_str());

    std::cout << "SOR filter for: " << filename.c_str() << std::endl;
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>(filename, *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    const clock_t begin_time = clock();
    sor.setInputCloud(cloud);
    sor.setMeanK(MeanK);
    sor.setStddevMulThresh(stddevMultiTresh);
    sor.filter(*cloud_filtered);
    std::cout << "time:" << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    pcl::io::savePCDFile(file_out,  *cloud_filtered, true);

    return (0);
}