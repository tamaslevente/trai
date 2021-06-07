#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>

using namespace cv;

int main(int argc, char **argv)
{
    int chars=atoi(argv[2]);
    std::string filename = argv[1];
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
    filename = filename.substr(0, filename.size() - chars);
    imwrite(filename + ".png", image);
    return 0;
}