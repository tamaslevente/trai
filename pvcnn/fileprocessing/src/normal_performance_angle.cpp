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

float compare(int cnt, std::string gtdir, std::string preddir, std::string gtending, std::string predending)
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
        return (-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointNormal>(file_pred, *cloudpred) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    std::cout << "PCD_GT has: " << cloudgt->size() << " data points." << std::endl;     //*
    std::cout << "PCD_PRED has: " << cloudpred->size() << " data points." << std::endl; //*
    float quality;
    float cloud_angle = 0;
    int histo[100] = {0};
    float rms = 0;
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
            bool nan = false;
            if (quality != quality || std::isnan(quality))
                nan = true;
            if (!nan)
            {
                if (quality < 0)
                {
                    quality = -quality;
                }
                float angle = pow(acos(quality) * 180 / 3.14159265, 2);
                cloud_angle += angle;
            }
        }
        cloud_angle = cloud_angle / cloudpred->size();
        rms += sqrt(cloud_angle);
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
            bool nan = false;
            if (quality != quality || std::isnan(quality))
                nan = true;
            if (!nan)
            {
                if (quality < 0)
                {
                    quality = -quality;
                }
                float angle = pow(acos(quality) * 180 / 3.14159265, 2);
                cloud_angle += angle;
            }
        }
        cloud_angle = cloud_angle / cloudpred->size();
        rms += sqrt(cloud_angle);
    }
    // return cloud_quality;
    return rms;
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

    float RMS = 0;
    int count = start;
    int cnt = 0;
    int histogram[1000] = {};
    ofstream myfile;
    myfile.open(histoname);

    while (count < end)
    {
        float rms = compare(count, gtdir, preddir, gtending, predending);
        int ind = (int)(rms * 10);
        if (ind >= 1000)
            ind = 999;
        histogram[ind]++;
        printf("RMS for %05d pcd is: %f\n", count, rms);
        RMS += rms;
        count = count + step;
        cnt++;
    }
    float R = RMS / cnt;
    printf("The TOTAL quality for %d pointclouds is: %f\n\n", cnt, R);
    printf("The histogram:\n");
    for (int i = 0; i < 1000; i++)
    {
        if (histogram[i] > 0)
        {
            myfile << i << " " << histogram[i] << "\n";
            printf("%03f:%04d - ", i / 10, histogram[i]);
            for (int j = 0; j < histogram[i]; j++)
                printf("|");
            printf("\n");
        }
    }
    myfile.close();
    return 0;
}
