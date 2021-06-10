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
    cv::Scalar detectionColor = cv::Scalar(255,165,0);


    std::vector<std::vector<cv::Point>> contours;       //contours are stored here
    std::vector<cv::Vec4i>              hierarchy;                   
    


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
        
        double maxArea = 0;
        int maxAreaContourId = -1;

        for ( size_t i = 0; i< contours.size();i++)
        {
            double newArea = cv::contourArea(contours.at(i));
            if (newArea > maxArea)
            {
                maxArea = newArea;
                maxAreaContourId = i;
            }
        }
        
        if (maxAreaContourId>0)
        {   

            std::vector<cv::Point> contour_poly   (contours.size());
            cv::Point2f            center         (contours.size());  
            float                  radius          (contours.size());

            cv::approxPolyDP(contours[maxAreaContourId], contour_poly, 3, true);
            cv::minEnclosingCircle(contour_poly, center, radius);

            // uncomment the following to draw contours exactly
            cv::drawContours(drawing, contours, maxAreaContourId, detectionColor ,5, cv::LINE_8, hierarchy, 0 );

            cv::circle(drawing, center, int(radius), detectionColor, 2 );

            // uncomment the following for checking center coordinates
            // std::cout<<center<<'\n';
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