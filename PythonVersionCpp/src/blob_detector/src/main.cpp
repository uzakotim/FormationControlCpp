// include ros library
#include <ros/ros.h>
// include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";


// Topics
static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
static const std::string PUBLISH_TOPIC = "/image_converter/output_video";


void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    ROS_INFO_STREAM("[Image from: " << frame_id<<" ]\n");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // Update GUI Window
        ROS_INFO_STREAM("Hello world\n");
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    // Draw an example crosshair
    // cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
    // cv::waitKey(3);

    // Output modified video stream
    // pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roscpp_open_cv");
    //Publisher/Subscriber 
    ros::NodeHandle nh;
    cv::namedWindow("live");
    
    ROS_INFO_STREAM("Hello world before\n");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, image_callback);
    ROS_INFO_STREAM("Hello world after\n");
    // ros::Publisher pub = nh.advertise<sensor_msgs::Image>(PUBLISH_TOPIC, 10);
    // ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC,10,image_callback);

    ros::spin();
    cv::destroyWindow("live");
    // ROS_INFO_STREAM("Hello from ROS node: " << ros::this_node::getName());



    return 0;

}
