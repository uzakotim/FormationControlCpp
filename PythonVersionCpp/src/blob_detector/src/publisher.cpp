#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{   
    if(argv[1] == NULL) return 1; // Check if video source has been passed as a parameter


    ros::init(argc,argv, "image_publisher");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello from Camera publisher node");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image",1);
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    int api_id = cv::CAP_ANY;
    if(!(video_sourceCmd>>video_source)) return 1;
    

    cv::VideoCapture cap(video_source, api_id);
    if (!cap.isOpened()) return 1;


    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    
    ros::Rate loop_rate(100);
    int count_ {0};
    while(nh.ok())
    {   
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()) 
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            msg->header.frame_id = count_;
            pub.publish(msg);
            count_++;
            cv::waitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}