#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    char directory[200] = "/media/rambo/ssd2/Szilard/file_repository/4bag_unfiltered/";
    char file_depth[200];
    char file_ir[200];
    char file_depth2ir[200];
    int cnt = 0;

    while (cnt < 4330)
    {
        sprintf(file_depth, "%snoisydepth/%05d_depth_noise_05.png", directory, cnt);
        sprintf(file_depth2ir, "%snoisydepth2ir/%05d_depth2ir.png", directory, cnt);
        sprintf(file_ir, "%sir/%05d_ir.png", directory, cnt);
        cv::Mat mat_depth = cv::imread(file_depth, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat mat_ir = cv::imread(file_ir, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat zerochannel = cv::Mat::zeros(cv::Size(mat_depth.rows, mat_depth.cols), CV_16U);
        cv::Mat output = cv::Mat::zeros(mat_depth.rows, mat_depth.cols, CV_16UC3);
        cv::Mat images[3] = {mat_ir, mat_depth, mat_depth};
        int dims[3] = {2, mat_depth.rows, mat_depth.cols};
        cv::Mat joined(3, dims, CV_16U);
        for (int i = 0; i < 3; i++)
        {
            uint16_t *ptr = &joined.at<uint16_t>(i, 0, 0);                            // pointer to first element of slice i
            cv::Mat destination(mat_depth.rows, mat_depth.cols, CV_32S, (void *)ptr); // no data copy, see documentation
            images[i].copyTo(destination);
        }

        for (int x = 0; x < images[0].rows; x++)
        {
            for (int y = 0; y < images[0].cols; y++)
            {
                output.at<cv::Vec3s>(x, y)[2] = images[0].at<unsigned short>(x, y);
                output.at<cv::Vec3s>(x, y)[1] = images[1].at<unsigned short>(x, y);
                output.at<cv::Vec3s>(x, y)[0] = images[2].at<unsigned short>(x, y);
            }
        }
        cv::imwrite(file_depth2ir, output);
        cnt++;
    }

    return (0);
}
