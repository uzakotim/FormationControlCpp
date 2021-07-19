// Copyright [2021] [Timur Uzakov]

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>
#include <cmath>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class Tracker
{

private:
    ros::NodeHandle nh;
    ros::Publisher human_pub;

    message_filters::Subscriber<PoseWithCovarianceStamped> human_sub;
    message_filters::Subscriber<Odometry> pose_sub;
    typedef sync_policies::ApproximateTime<PoseWithCovarianceStamped,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    std::string human_sub_topic = "/uav1/human";
    std::string pose_sub_topic  = "/uav1/odometry/odom_main";
    std::string human_pub_topic = "/uav1/tracker";
    
    PoseWithCovarianceStamped msg;
    int count {0};
public:
    cv::Mat state, human_coord,human_cov,human_world;
    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);

    double yaw_value;
    Tracker()
    {
       
        human_sub.subscribe(nh,human_sub_topic,1);
        pose_sub.subscribe(nh,pose_sub_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), human_sub,pose_sub));
        sync->registerCallback(boost::bind(&Tracker::callback,this,_1,_2));

        human_pub = nh.advertise<PoseWithCovarianceStamped>(human_pub_topic,1);

        ROS_INFO("All functions initialized");
    }
   
    cv::Mat HumanCoordinateToWorld(cv::Mat object_position,double yaw_value,cv::Mat drone_position,cv::Mat offset_vector)
    {
        cv::Mat shift_to_center     = (cv::Mat_<float>(3,1) << 1280/2,720/2,0); //!TODO! Check image size
        cv::Mat scale_matrix        = (cv::Mat_<float>(3,3) << 0.005,0,0,  0,-0.005,0,     0,0,1); //x same, y flip and rescale
        // uncomment for debugging 
        // std::cout<<shift_to_center<<'\n';
        cv::Mat shifted_and_scaled  = scale_matrix*(object_position - shift_to_center);
        cv::Mat R                   = (cv::Mat_<float>(3,3) << sin(yaw_value),0,cos(yaw_value),    -cos(yaw_value),0,sin(yaw_value) ,   0,1,0);
        cv::Mat rotated_vector      = R*shifted_and_scaled;

        cv::Mat point = drone_position + offset_vector + rotated_vector;
        
        return point;
    }

    void callback(const PoseWithCovarianceStampedConstPtr& human,const OdometryConstPtr& pose)
    {
        ROS_INFO("Synchronized\n");
        state = (cv::Mat_<float>(4,1)<< pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z,pose->pose.pose.orientation.z);
        human_coord = (cv::Mat_<float>(3,1)<< human->pose.pose.position.x,human->pose.pose.position.y,human->pose.pose.position.z);
        yaw_value = pose->pose.pose.orientation.z;
        cv::Mat offset = (cv::Mat_<float>(3,1) << (0.2*cos(yaw_value)),(0.2*sin(yaw_value)),0); 
        human_world = HumanCoordinateToWorld(human_coord,yaw_value,state,offset);
        ROS_INFO_STREAM(human_world);

        msg.header.frame_id = count;
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = human_world.at<float>(0);
        msg.pose.pose.position.y = human_world.at<float>(1);
        msg.pose.pose.position.z = human_world.at<float>(2);
        msg.pose.covariance = human->pose.covariance;
        ros::Rate rate(500);
        human_pub.publish(msg);
        rate.sleep();
        count++;

        
    }

};     



int main(int argc, char** argv)
{
    ROS_INFO("Tracker node initialized");
    ros::init(argc, argv, "tracker_node");
    Tracker tr;
    ros::spin();


    return 0;
}
