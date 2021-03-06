// include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// include ros library
#include <ros/ros.h>

class BlobDetector 
{
private:
    image_transport::Publisher pub;
    image_transport::Subscriber sub;
    int count = 0;
    sensor_msgs::ImagePtr msg_output;
public:
    BlobDetector(ros::NodeHandle *nh)
    {   
        image_transport::ImageTransport it(*nh);
        pub = it.advertise("camera/blob", 1);
        sub = it.subscribe("camera/image", 1, &BlobDetector::image_callback,this);

    }
    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        std_msgs::Header    msg_header = msg->header;
        std::string         frame_id = msg_header.frame_id;
        ROS_INFO_STREAM("[Image from: " << frame_id<<" ]");
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        cv::Mat imageHSV;
        cv::cvtColor(image, imageHSV,CV_BGR2HSV);

        msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageHSV).toImageMsg();
        msg_output->header.frame_id = std::to_string(count);
        count++;
        pub.publish(msg_output);
        ROS_INFO_STREAM("[Image:" << frame_id<<" was sent ]");
    }
};




int main(int argc, char** argv)

{
    ROS_INFO_STREAM  ("Instanciating Blob Detector\n");
    ros::init        (argc, argv, "roscpp_open_cv");
    ros::NodeHandle  nh;
    BlobDetector     bd = BlobDetector(&nh);
    ros::spin();
    return 0;
}
