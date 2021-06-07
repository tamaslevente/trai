#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
    std::string gtdir = argv[1];
    std::string preddir = argv[2];
    std::string outdir = argv[3];
    std::string gtfilename = argv[4];
    std::string predfilename = argv[5];

    char file_gt[200];
    char file_pred[200];
    char file_out[200];

    sprintf(file_gt, "%s%s", gtdir.c_str(), gtfilename.c_str());
    std::cout << "Processing: " << file_gt << std::endl;
    sprintf(file_pred, "%s%s", preddir.c_str(), predfilename.c_str());
    sprintf(file_out, "%s%s", outdir.c_str(), gtfilename.c_str());

    cv::Mat gt = cv::imread(file_gt, cv::IMREAD_GRAYSCALE);
    cv::Mat pred = cv::imread(file_pred, cv::IMREAD_GRAYSCALE);
    cv::Mat output = cv::Mat::zeros(gt.rows, gt.cols, CV_8UC3);

    for (int i = 0; i < gt.rows; i++)
    {
        for (int j = 0; j < gt.cols; j++)
        {
            float d = gt.at<uint8_t>(i, j);
            float m = pred.at<uint8_t>(i, j);
            output.at<cv::Vec3b>(i, j)[0] = 0;
            output.at<cv::Vec3b>(i, j)[1] = 0;
            output.at<cv::Vec3b>(i, j)[2] = 0;
            if (d > 0 && m > 0)
            {
                output.at<cv::Vec3b>(i, j)[1] = 255;
            }
            else
            {
                if (d > 0 && m == 0)
                {
                    output.at<cv::Vec3b>(i, j)[1] = 255;
                }
                else
                {
                    if (d == 0 && m > 0)
                    {
                        output.at<cv::Vec3b>(i, j)[2] = 255;
                    }
                }
            }
        }
    }
    cv::imwrite(file_out, output);

    return 0;
}
