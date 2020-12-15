
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>

using namespace cv;

int main()
{
    std::string filename = "";
    std::cin >> filename;
    cv::Mat image = cv::imread(filename);
    filename = filename.substr(0, filename.size() - 3);
    imwrite(filename + "png", image);
    return 0;
}