
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
    std::string gtdir = argv[1];
    std::string preddir = argv[2];
    std::string gtending = argv[3];
    std::string predending = argv[4];
    char file_gt[200];
    char file_pred[200];
    int start = atoi(argv[5]);
    int end = atoi(argv[6]);
    int step = atoi(argv[7]);
    int count = start;
    int cnt = 0;
    float avg_missing = 0;
    float avg_extra = 0;
    float avg_bothzero = 0;
    float avg_bothvalue = 0;
    float avg_difference = 0;

    while (count < end)
    {

        sprintf(file_gt, "%s%05d%s", gtdir.c_str(), count, gtending.c_str());
        sprintf(file_pred, "%s%05d%s", preddir.c_str(), count, predending.c_str());
        cv::Mat mat_depthgt = cv::imread(file_gt, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat mat_depthpred = cv::imread(file_pred, CV_LOAD_IMAGE_UNCHANGED);
        std::cout << file_gt << std::endl;
        int missing = 0;
        int extra = 0;
        int bothzero = 0;
        int bothvalue = 0;
        float difference = 0;
        float th = 100;
        for (int x = 0; x < mat_depthgt.rows; x++)
        {
            for (int y = 0; y < mat_depthgt.cols; y++)
            {

                if (mat_depthgt.at<uint16_t>(x, y) < th && mat_depthpred.at<uint16_t>(x, y) >= th)
                {
                    extra++;
                }
                else
                {
                    if (mat_depthgt.at<uint16_t>(x, y) >= th && mat_depthpred.at<uint16_t>(x, y) < th)
                    {
                        missing++;
                    }
                    else
                    {
                        if (mat_depthgt.at<uint16_t>(x, y) < th && mat_depthpred.at<uint16_t>(x, y) < th)
                        {
                            bothzero++;
                        }
                        else
                        {
                            bothvalue++;
                            difference += abs(mat_depthgt.at<uint16_t>(x, y) - mat_depthpred.at<uint16_t>(x, y));
                        }
                    }
                }
            }
        }
        avg_missing += missing;
        avg_extra += extra;
        avg_bothzero += bothzero;
        avg_bothvalue += bothvalue;
        avg_difference += difference / (mat_depthgt.rows * mat_depthgt.cols - extra - missing);
        count = count + step;
        cnt++;
    }

    avg_missing /= cnt;
    avg_extra /= cnt;
    avg_bothzero /= cnt;
    avg_bothvalue /= cnt;
    avg_difference /= cnt;
    int prednr = avg_bothvalue + avg_extra;
    int gtnr = avg_bothvalue + avg_missing;
    float percent = ((float)prednr / (float)gtnr) * 100.0;
    std::cout << "Avg. missing: " << avg_missing << std::endl;
    std::cout << "Avg. extra: " << avg_extra << std::endl;
    std::cout << "Avg. difference: " << avg_difference << std::endl;
    std::cout << "Avg. both value: " << avg_bothvalue << std::endl;
    std::cout << "Prediction has  " << percent << " percent points from the gt in average." << std::endl;

    return (0);
}
