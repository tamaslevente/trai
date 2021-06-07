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
    std::string ddir = argv[1];
    std::string mdir = argv[2];
    std::string filename = argv[3];
    float multiplier = atof(argv[4]);
    char file_depth[200];
    char file_depthmask[200];

    sprintf(file_depth, "%s%s", ddir.c_str(), filename.c_str());
    filename = filename.substr(0, filename.size() - 4);
    std::cout << file_depth << std::endl;
    sprintf(file_depthmask, "%s%s_mask.png", mdir.c_str(), filename.c_str());

    cv::Mat mat_depth = cv::imread(file_depth, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat output = cv::Mat::zeros(mat_depth.rows, mat_depth.cols, CV_8UC1);

    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    int counter = 0;
    float th = 100;
    minMaxLoc(mat_depth, &minVal, &maxVal, &minLoc, &maxLoc);
    for (int x = 0; x < mat_depth.rows; x++)
    {
        for (int y = 0; y < mat_depth.cols; y++)
        {
            if (mat_depth.at<uint16_t>(x, y) >= th) // set threshold
            {
                counter++;
                output.at<uint8_t>(x, y) = 255;
            }
            else
            {
                output.at<uint8_t>(x, y) = 0;
            }
        }
    }
    std::cout << counter << std::endl;
    cv::imwrite(file_depthmask, output);

    return (0);
}
