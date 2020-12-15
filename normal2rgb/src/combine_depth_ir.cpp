
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>

using namespace cv;

int main()
{
    std::string directoryd = "/home/szilard/projects/normalrgb/depthir/depth/";
    std::string directoryi = "/home/szilard/projects/normalrgb/depthir/ir/";
    std::string directorys = "/home/szilard/projects/normalrgb/depthir/depthir/";
    std::string depth_filename = "depth_0.png";
    //std::cin >> depth_filename;
    std::string ir_filename = "ir_0.png";
    //std::cin >> ir_filename;
    cv::Mat depth_image = cv::imread(directoryd + depth_filename, IMREAD_GRAYSCALE);
    cv::Mat ir_image = cv::imread(directoryi + ir_filename, IMREAD_GRAYSCALE);
    cv::Mat zerochannel = cv::Mat::zeros(cv::Size(depth_image.rows, depth_image.cols), CV_8U);
    const int channels = depth_image.channels();
    printf("Number of channels = %d\n", channels);
    cv::Mat images[3] = {ir_image, depth_image, zerochannel};
    int dims[3] = {2, depth_image.rows, depth_image.cols};
    cv::Mat joined(3, dims, CV_8U);
    for (int i = 0; i < 3; ++i)
    {
        uint8_t *ptr = &joined.at<uint8_t>(i, 0, 0);                                 // pointer to first element of slice i
        cv::Mat destination(depth_image.rows, depth_image.cols, CV_8U, (void *)ptr); // no data copy, see documentation
        images[i].copyTo(destination);
    }
    cv::Mat output=cv::Mat::zeros(images[0].rows, images[0].cols, CV_8UC3);
    
    

    for (int x = 0; x < images[0].rows; x++)
    {
        for (int y = 0; y < images[0].cols; y++)
        {
            //printf("%d ", images[1].at<uchar>(x, y));
            output.at<cv::Vec3b>(x,y)[2] = images[0].at<uchar>(x, y);
            output.at<cv::Vec3b>(x,y)[1] = images[1].at<uchar>(x, y);
        }
        //printf("\n");
    }

    //cv::imshow("Display window", output);
    //waitKey(0); // Wait for a keystroke in the window

    imwrite(directorys + "depthir_0.png", output);

    return 0;
}