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
    std::string file_in = argv[1] + filename;
    std::string file_out = argv[3] + filename ;
    std::cout << argv[4] << std::endl;
    std::cout << file_out << std::endl;
    cv::Mat mat_in = cv::imread(file_in, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat noise = cv::Mat::zeros(mat_in.rows, mat_in.cols, CV_8UC3);
    cv::Mat mask = cv::Mat::zeros(mat_in.rows, mat_in.cols, CV_8UC3);
    mask = mat_in.mul(1 / mat_in);
    
    cv::Mat mean = cv::Mat::zeros(1, 1, CV_8UC3);
    cv::Mat sigma = cv::Mat::ones(1, 1, CV_8UC3);
    
    cv::Mat negativ_bias = cv::Mat::ones(mat_in.rows, mat_in.cols, CV_8UC3);
    float rate = atof(argv[4]);    
    int s= 255*rate;
    std::cout<<s<<std::endl;
    cv::randn(noise, s, s);
    printf("%u+", mat_in.at<uint8_t>(100, 100));
    printf("%u=", noise.at<uint8_t>(100, 100));
    cv::Mat output = cv::Mat::zeros(mat_in.rows, mat_in.cols, CV_8UC3);
    output = mat_in + noise.mul(mask) - negativ_bias.mul(mask) * s;
    printf("%u\n", output.at<uint8_t>(100, 100));
    cv::imwrite(file_out, output);

    return (0);
}
