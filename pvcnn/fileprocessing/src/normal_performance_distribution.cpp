#include <boost/algorithm/string.hpp>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

using namespace std;

double threshold[7] = {0.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0}; //degree
double percentage[7] = {};

void compare(int cnt, std::string gtdir, std::string preddir, std::string gtending, std::string predending)
{

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudpred(new pcl::PointCloud<pcl::PointNormal>);
    char file_gt[200];
    char file_pred[200];
    sprintf(file_gt, "%s%05d%s", gtdir.c_str(), cnt, gtending.c_str());
    sprintf(file_pred, "%s%05d%s", preddir.c_str(), cnt, predending.c_str());
    if (pcl::io::loadPCDFile<pcl::PointNormal>(file_gt, *cloudgt) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return;
    }

    if (pcl::io::loadPCDFile<pcl::PointNormal>(file_pred, *cloudpred) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return;
    }

    std::cout << "PCD_GT has: " << cloudgt->size() << " data points." << std::endl;     //*
    std::cout << "PCD_PRED has: " << cloudpred->size() << " data points." << std::endl; //*
    double quality;
    double cloud_quality[7] = {};
    int less_thr[7] = {};
    if (cloudgt->size() < cloudpred->size())
    {
        for (int i = 0; i < cloudgt->size(); i++)
        {
            Eigen::Vector3f normalgt;
            normalgt << cloudgt->points[i].normal_x, cloudgt->points[i].normal_y, cloudgt->points[i].normal_z;
            normalgt.normalize();
            Eigen::Vector3f normalpred;
            normalpred << cloudpred->points[i].normal_x, cloudpred->points[i].normal_y, cloudpred->points[i].normal_z;
            normalpred.normalize();
            quality = normalgt.dot(normalpred);
            // double angle = acos(quality);
            bool nan = false;
            if (quality != quality || std::isnan(quality))
                nan = true;
            if (!nan)
            {
                if (quality < 0)
                {
                    quality = -quality;
                }
                // cloud_quality = cloud_quality + quality;
                double angle = acos(quality);
                // printf("%f\n",angle);
                for (int i = 0; i < 7; i++)
                {
                    if (angle < threshold[i])
                        less_thr[i]++;
                }
            }
        }
        for (int i = 0; i < 7; i++)
        {
            cloud_quality[i] = (double)less_thr[i] / cloudgt->size();
            percentage[i] += cloud_quality[i];
        }
    }
    else
    {
        for (int i = 0; i < cloudpred->size(); i++)
        {
            Eigen::Vector3f normalgt;
            normalgt << cloudgt->points[i].normal_x, cloudgt->points[i].normal_y, cloudgt->points[i].normal_z;
            normalgt.normalize();
            Eigen::Vector3f normalpred;
            normalpred << cloudpred->points[i].normal_x, cloudpred->points[i].normal_y, cloudpred->points[i].normal_z;
            normalpred.normalize();
            quality = normalgt.dot(normalpred);
            // double angle = acos(quality);
            bool nan = false;
            if (quality != quality || std::isnan(quality))
                nan = true;
            if (!nan)
            {
                if (quality < 0)
                {
                    quality = -quality;
                }
                // cloud_quality = cloud_quality + quality;
                double angle = acos(quality);
                // printf("%f -- %f\n", angle, threshold);
                for (int i = 0; i < 7; i++)
                {
                    if (angle < threshold[i])
                        less_thr[i]++;
                }
            }
        }
        for (int i = 0; i < 7; i++)
        {
            cloud_quality[i] = (double)less_thr[i] / cloudpred->size();
            percentage[i] += cloud_quality[i];
        }
    }
}

int main(int argc, char *argv[])
{
    for (int i = 0; i < 7; i++)
    {
        threshold[i] = threshold[i] * 3.14 / 180;
    }
    std::string gtdir = argv[1];
    std::string preddir = argv[2];
    std::string gtending = argv[3];
    std::string predending = argv[4];
    int start = atoi(argv[5]);
    int end = atoi(argv[6]);
    int step = atoi(argv[7]);
    int count = start;
    int cnt = 0;
    while (count < end)
    {
        compare(count, gtdir, preddir, gtending, predending);
        count = count + step;
        cnt++;
    }
    printf("The percentages are: \n");
    for (int i = 0; i < 7; i++)
    {
        double P = percentage[i] / cnt;
        printf("%f, ", P * 100);
    }
    printf("\b\b\n");
    return 0;
}
