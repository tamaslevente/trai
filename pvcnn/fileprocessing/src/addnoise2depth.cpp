#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

using namespace std;
using namespace cv;

/*
argv:
1 - input - depth folder path
2 - input - filename
3 - output - noise depth folder path
4 - output - noise depth extension
5 - sigma
*/

int main(int argc, char **argv)
{

    std::string filename = argv[2];
    std::string file_depth = argv[1] + filename;
    filename = filename.substr(0, filename.size() - 4);
    std::string file_depthn = argv[3] + filename + "_" + argv[4] + ".png";
    std::cout << argv[4] << std::endl;
    std::cout << file_depthn << std::endl;
    cv::Mat mat_depth = cv::imread(file_depth, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat noise = cv::Mat::zeros(mat_depth.rows, mat_depth.cols, CV_16UC1);
    cv::Mat mask = cv::Mat::zeros(mat_depth.rows, mat_depth.cols, CV_16UC1);
    mask = mat_depth.mul(1 / mat_depth);
    cv::Mat mean = cv::Mat::zeros(1, 1, CV_16UC1);
    cv::Mat sigma = cv::Mat::ones(1, 1, CV_16UC1);
    cv::Mat negativ_bias = cv::Mat::ones(mat_depth.rows, mat_depth.cols, CV_16UC1);
    int s = 4 * atoi(argv[4]);
    cv::randn(noise, mean + s, sigma * s / 4);
    printf("%u+", mat_depth.at<uint16_t>(100, 100));
    printf("%u=", noise.at<uint16_t>(100, 100));
    cv::Mat output = cv::Mat::zeros(mat_depth.rows, mat_depth.cols, CV_16UC1);
    output = mat_depth + noise.mul(mask) - negativ_bias.mul(mask) * s;
    printf("%u\n", output.at<uint16_t>(100, 100));
    cv::imwrite(file_depthn, output);

    return (0);
}
