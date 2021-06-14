#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/video/tracking.hpp>




using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
    public:
        Publisher()
        :   Node("blob_detector"), count_(0)
        {
            publisher_= this->create_publisher<sensor_msgs::ImagePtr>("camera/blob_detector",10);
            timer_ = this->create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
        }


    private:
        cv::VideoCapture cap;
        cv::Mat frame;
        int deviceID=0;
        // open the default camera using default API
        // cap.open(0);
        // or
        int device_id   = 0;            // 0 = open default camera
        int api_id      = cv::CAP_ANY;      // 0 = autodetect default API

        cv::Scalar                          orange_min = cv::Scalar(10,150,150);     //min hsv value orange
        cv::Scalar                          orange_max = cv::Scalar(27,255,255);     //max hsv value orange
        cv::Scalar                          detection_color = cv::Scalar(255,100,0);
        
        cap.open(device_id,api_id);
    
        if (!cap.isOpened())            // check if succeded
        {
            std::cerr<< "ERROR! Unable to open camera \n";
            return -1;
        } 


        void timer_callback()
        {   
            cap.read(frame);
            if  (frame.empty())          // check if succeded
            {
                std::cerr<<"ERROR! blank frame grabbed\n";
                break;
            }
            else
            {
                sensor_msgs::ImagePtr message = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
                publisher_->publish(message);
            }
            
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}