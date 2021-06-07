#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    std::string filename = argv[1];
    std::cout << filename << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointNormal>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    pcl::io::savePCDFile(filename, *cloud, true);
    return (0);
}
