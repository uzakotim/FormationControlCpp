#include <stdio.h>
#include <opencv2/opencv.hpp>


int main(int argc, char** argv)
{
    cv::Mat image;
    cv::VideoCapture cap;

    // open the default camera using default API
    // cap.open(0);
    // or
    int deviceID = 0;            // 0 = open default camera
    int apiID= cv::CAP_ANY;      // 0 = autodetect default API

    cap.open(deviceID,apiID);
    // check if succeded
    if (!cap.isOpened())
    {
        std::cerr<< "ERROR! Unable to open camera \n";
        return -1;
    } 

    std::cout<<"Start grabbing"<<'\n'<<"Press any key to terminate"<<'\n';
    
    for (;;)
    {
        cap.read(image);
        if (image.empty())          // check if succeded
        {
            std::cerr<<"ERROR! blank frame grabbed\n";
            break;
        }    
        cv::namedWindow("Live", cv::WINDOW_AUTOSIZE);
        cv::imshow("Live", image);  // show image
        if (cv::waitKey(5) >= 0)    // break if a key is pressed
            break;

    }

    return 0;
}