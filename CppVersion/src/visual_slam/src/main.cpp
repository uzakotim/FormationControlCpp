// Copyright [2021] [Timur Uzakov]

#include <iostream>

// Time Synchronizer Libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Libraries for msgs
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// CvBridge, Image Transport
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV libraries
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

// Namespaces
using namespace geometry_msgs;
using namespace message_filters;
using namespace sensor_msgs;

class SLAM
{               /*--------------VISUAL SLAM NODE-----------------*/
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    sensor_msgs::ImagePtr msg_output;

    
    // Time Synchronizer variables ---------
    message_filters::Subscriber<PointStamped>   sub_1;
    message_filters::Subscriber<Image>          sub_2;
    typedef sync_policies::ApproximateTime<PointStamped,Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync>                     sync;
    // Time Synchronizer variables --------- 

    // Msgs Topics -------------------------
    std::string sub_goal_topic      = "/computations/goal_point";
    std::string sub_camera_topic    = "/camera/image"; 
    std::string pub_image_topic     = "/slam/matches";
    // Msgs Topics -------------------------

    // Visual SLAM variables ---------------
    int count {0};
    cv::Mat image, image_prev, image_matches;
    const int   MAX_FEATURES        = 500;
    const float GOOD_MATCH_PERCENT  = 0.15f;
    bool        flag_first_photo    = true;

    cv::Mat                     descriptors;
    cv::Mat                     descriptors_prev;
    std::vector<cv::KeyPoint>   keypoints;
    std::vector<cv::KeyPoint>   keypoints_prev;
    
    cv::Ptr<cv::Feature2D>          orb     = cv::ORB::create(MAX_FEATURES);
    cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch>         matches;
    // Visual SLAM variables ---------------
   
public:
    SLAM()
    {
    //  SLAM Node Constructor --------------
        
        sub_1.subscribe(nh,sub_goal_topic,1);
        sub_2.subscribe(nh,sub_camera_topic,1);
        
        pub = nh.advertise<sensor_msgs::Image>(pub_image_topic,1);
    //  SLAM Node Constructor --------------


    // Parameters for Time Synchronizer, two subscribers connected to one SLAM::callback
        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2));
        sync->registerCallback(boost::bind(&SLAM::callback,this,_1,_2));
        
        ROS_INFO("SLAM Node Initialized Successfully");
    }
    // Parameters for Time Synchronizer, two subscribers connected to one SLAM::callback

    cv::Mat ReturnCVMatImageFromMsg(const sensor_msgs::ImageConstPtr& msg)
    {
        // Function that converts Image msg into cv::Mat format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            std::cerr<<"Could not convert image into CV Mat\n";
        }
        cv::Mat image = cv_ptr->image;
        return  image;
    }

    void callback(const PointStampedConstPtr& point,const ImageConstPtr& msg)
    {
        // Callback of SLAM Node

        ROS_INFO("[Synchronized and SLAM started]\n");

        cv::Mat image  = ReturnCVMatImageFromMsg (msg);

        if (flag_first_photo == true) 
        {   
            // If the frame is taken for the first time, 
            // compute keypoints and descriptors using ORB,
            // clone image frame for output,
            // set the flag to false
            orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            
            image_matches           = image.clone();
            flag_first_photo        = false;

            ROS_INFO("First Photo\n");
            
        }
        else
        {  
            // If the frame is not first, compute current frame,keypoints and descriptors,
            // match with previously stored keypoints and descriptors,
            // sort and draw them
            
            orb     ->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            matcher ->match(descriptors_prev, descriptors, matches, cv::Mat());
            
            std::sort(matches.begin(), matches.end()); //sort matches by score
            const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
            matches.erase(matches.begin()+numGoodMatches,matches.end()); //remove bad matches

            cv::drawMatches(image_prev, keypoints_prev, image,keypoints, matches, image_matches);
        }

        // Preparation of output msg
        msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matches).toImageMsg();
        msg_output->header.frame_id = std::to_string(count);
        msg_output->header.stamp= ros::Time::now();

        // Publishing
        pub.publish(msg_output);
       
        // frame update for ORB detection of features
        image_prev          = image.clone();
        keypoints_prev      = keypoints;
        descriptors_prev    = descriptors;
        count++;
    }


};     



int main(int argc, char** argv)
{
    ROS_INFO("VisualSLAM node initialized");
    ros::init(argc, argv, "vision_node");
    SLAM vs;
    ros::spin();

    return 0;
}