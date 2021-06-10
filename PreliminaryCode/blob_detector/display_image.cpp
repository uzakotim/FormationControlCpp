#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/hal/interface.h>

int main(int argc, char** argv)
{
    cv::Mat frame, frame_HSV,frame_threshold;
    cv::VideoCapture cap;

    // open the default camera using default API
    // cap.open(0);
    // or
    int deviceID = 0;            // 0 = open default camera
    int apiID= cv::CAP_ANY;      // 0 = autodetect default API

    cv::Scalar ORANGE_MIN = cv::Scalar(10,150,150);     //min hsv value orange
    cv::Scalar ORANGE_MAX = cv::Scalar(27,255,255);     //max hsv value orange

    std::vector<std::vector<cv::Point>> contours;       //contours are stored here
    std::vector<cv::Vec4i> hierarchy;                   

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
        cap.read(frame);
        if (frame.empty())          // check if succeded
        {
            std::cerr<<"ERROR! blank frame grabbed\n";
            break;
        } 
        cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
        cv::inRange(frame_HSV, ORANGE_MIN, ORANGE_MAX,frame_threshold);

        cv::findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat drawing = cv::Mat::zeros(frame_threshold.size(), CV_8UC3 );
        for ( size_t i = 0; i< contours.size();i++)
        {
            cv::drawContours(drawing, contours, (int)i, cv::Scalar(255,165,0),5, cv::LINE_8, hierarchy, 0 );
        }
        cv::namedWindow("Live", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Live Mask", cv::WINDOW_AUTOSIZE);
        cv::imshow("Live", frame + drawing);  // show image
        cv::imshow("Live Mask", frame_threshold);
        
        
        if (cv::waitKey(5) >= 0)    // break if a key is pressed
            break;

    }

    return 0;
}