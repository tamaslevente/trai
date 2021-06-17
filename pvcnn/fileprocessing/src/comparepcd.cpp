#include <boost/algorithm/string.hpp>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <string>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;

float compare(int cnt, std::string gtdir, std::string preddir, std::string gtending, std::string predending)
{
    Cloud::Ptr cloudgt(new Cloud);
    Cloud::Ptr cloudpred(new Cloud);

    char file_gt[200];
    char file_pred[200];
    char file_delta[200];

    sprintf(file_gt, "%s%05d%s", gtdir.c_str(), cnt, gtending.c_str());
    sprintf(file_pred, "%s%05d%s", preddir.c_str(), cnt, predending.c_str());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_gt, *cloudgt) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_pred, *cloudpred) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    std::cout << "PCD_GT " << cnt << " has: " << cloudgt->size() << " data points." << std::endl; //*
    std::cout << "PCD_PRED has: " << cloudpred->size() << " data points." << std::endl;           //*
    TicToc tt;
    tt.tic();
    pcl::search::KdTree<PointType> treepred;
    treepred.setInputCloud(cloudpred->makeShared());
    float max_dist_a = 0.0;
    for (size_t i = 0; i < cloudgt->points.size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        treepred.nearestKSearch(cloudgt->points[i], 1, indices, sqr_distances);
        max_dist_a = max_dist_a + sqr_distances[0];
    }
    max_dist_a = std::sqrt(max_dist_a / cloudgt->points.size());

    // compare B to A
    pcl::search::KdTree<PointType> treegt;
    treegt.setInputCloud(cloudgt->makeShared());
    float max_dist_b = 0;
    for (size_t i = 0; i < cloudpred->points.size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        treegt.nearestKSearch(cloudpred->points[i], 1, indices, sqr_distances);
        max_dist_b = max_dist_b + sqr_distances[0];
    }

    max_dist_a = std::sqrt(max_dist_a / cloudgt->points.size());
    max_dist_b = std::sqrt(max_dist_b / cloudpred->points.size());

    float dist = std::max(max_dist_a, max_dist_b);

    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms : ");
    print_info("A->B: ");
    print_value("%f", max_dist_a);
    print_info(", B->A: ");
    print_value("%f", max_dist_b);
    print_info(", Hausdorff Distance: ");
    print_value("%f", dist);
    print_info(" ]\n");
    return dist;
}

int main(int argc, char *argv[])
{
    std::string histoname = argv[1];
    std::string gtdir = argv[2];
    std::string preddir = argv[3];
    std::string gtending = argv[4];
    std::string predending = argv[5];
    
    int start = atoi(argv[6]);
    int end = atoi(argv[7]);
    int step = atoi(argv[8]);
    int count = start;
    int cnt = 0;
    float distance = 0;
    ofstream myfile;
    myfile.open(histoname);

    while (count < end)
    {
        float q = compare(count, gtdir, preddir, gtending, predending);
        // printf("%f\n",q);
        myfile << q << "\n";
        distance = distance + q;
        count = count + step;
        cnt++;
    }
    double D = distance / cnt;
    printf("The TOTAL distance for %d pointclouds is: %f\n\n", cnt, D);
    myfile.close();
    return 0;
}
