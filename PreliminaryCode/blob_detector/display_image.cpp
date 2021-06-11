#include <cstdio>
#include <ctime>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/video/tracking.hpp>

int FindMaxAreaContourId(std::vector<std::vector<cv::Point>> contours)
{
    double  maxArea = 0;
    int     maxAreaContourId = -1;

    for (size_t i = 0;i<contours.size();i++)
    {
        double newArea = cv::contourArea(contours.at(i));
        if  (newArea > maxArea)
        {
                maxArea = newArea;
                maxAreaContourId = i;
        }
    }
    return maxAreaContourId;
}

int main(int argc, char** argv)
{
    cv::Mat             frame,frame_HSV,frame_threshold;
    cv::VideoCapture    cap;

    // open the default camera using default API
    // cap.open(0);
    // or
    int device_id   = 0;            // 0 = open default camera
    int api_id      = cv::CAP_ANY;      // 0 = autodetect default API

    cv::Scalar                          orange_min = cv::Scalar(10,150,150);     //min hsv value orange
    cv::Scalar                          orange_max = cv::Scalar(27,255,255);     //max hsv value orange
    cv::Scalar                          detection_color = cv::Scalar(255,100,0);


    std::vector<std::vector<cv::Point>> contours;       //contours are stored here
    std::vector<cv::Vec4i>              hierarchy;                   
    
    cv::Point2f                         last_recognized_center_point{(0.f,0.f)};


    cap.open(device_id,api_id);
    
    if (!cap.isOpened())            // check if succeded
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
        cv::inRange(frame_HSV, orange_min, orange_max,frame_threshold);
        cv::findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat drawing = cv::Mat::zeros(frame_threshold.size(), CV_8UC3 );
        cv::Mat drawingKF = cv::Mat::zeros(frame_threshold.size(), CV_8UC3 );

        int maxAreaContourId = FindMaxAreaContourId(contours);
        
        // uncomment to test FindMaxAreaContourId function
        // std::cout<<maxAreaContourId<<'\n';


      
        if (maxAreaContourId>=0)
        {   
            

            std::vector<cv::Point> contour_poly   (contours.size());
            cv::Point2f            center         (contours.size());  
            float                  radius          (contours.size());

            cv::approxPolyDP(contours[maxAreaContourId], contour_poly, 3, true);
            cv::minEnclosingCircle(contour_poly, center, radius);

            // uncomment the following to draw contours exactly
            // cv::drawContours(drawing, contours, maxAreaContourId, detectionColor ,5, cv::LINE_8, hierarchy, 0 );

            cv::circle(drawing, center, int(radius), detection_color, 2 );
            cv::circle(drawing, center, 5, detection_color, 10);

            // uncomment the following for checking center coordinates
            // std::cout<<center<<'\n';
        }
        

                
        cv::namedWindow("Live", cv::WINDOW_AUTOSIZE);
        
        cv::imshow("Live", frame + drawing);  // show image
        
        // uncomment the following to see mask separately
        // cv::namedWindow("Live Mask", cv::WINDOW_AUTOSIZE);
        // cv::imshow("Live Mask", frame_threshold);
        
        
        if (cv::waitKey(5) >= 0)    // break if a key is pressed
            break;

    }

    return 0;
}