// include message filters and time sync
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// include ros library
#include <ros/ros.h>
#include <cmath>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

class FormationController 
{
private:

    message_filters::Subscriber<_Float32[]> sub_1;
    message_filters::Subscriber<Odometry> sub_2;
    typedef sync_policies::ApproximateTime<_Float32[],Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // parameters
    double n_pos = 1.2;
    double n_neg = 0.5;
    double delta_max = 0.5;
    double delta_min = 0.000001;
    double radius = 6;

    ros::Publisher error_pub;
    std::string error_topic = "/uav1/error";
    ros::Publisher pose_pub;
    std::string pose_topic = "/uav1/control_manager/goto";

    PointStamped msg;

    cv::Mat tracker_vector = (cv::Mat_<float>(3,1) << 0,0,0);
    std::vector<cv::Mat> tracker;
    int count {0};
    cv::Mat w_prev = (cv::Mat_<float>(3,1) <<  0,0,0);
    cv::Mat master_pose;

    // measurements
    cv::Mat state,state_cov,human_coord,human_cov;

public:
    FormationController()
    {
        ros::NodeHandle nh;
        sub_1.subscribe(nh,"/uav1/sensor_fusion",1);
        sub_2.subscribe(nh,"/odometry/odom_main",1);
    
        //  TODO: Ensure that callback is called
        sync.reset              (new Sync(MySyncPolicy(10), sub_1, sub_2));
        sync->registerCallback  (boost::bind(&FormationController::callback, this, _1, _2));
        
        
        // TODO Ensure the sensor fusion message is transported and received
        
        error_pub = nh.advertise<_Float32[]>(error_topic, 1);
        pose_pub = nh.advertise<PointStamped>(pose_topic, 1);

        // measurements

        tracker.push_back(tracker_vector);
        tracker.push_back(tracker_vector);
        ROS_INFO("All functions initialized");


        
    }
    void callback(const PointStampedConstPtr& goal_point, const nav_msgs::OdometryPtr &pose)
    {
        // measurements
        state = (cv::Mat_<float>(4,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z,pose->pose.pose.orientation.z);
        state_cov = (cv::Mat_<float>(6,6) << pose->pose.covariance); // TODO Check if recevied
        // ROS_INFO("state covariance"<<state_cov<<'\n');
        human_coord = (cv::Mat_<float>(3,1) << goal_point->point.x, goal_point->point.y, goal_point->point.z);
        // human_cov = (cv::Mat_<float>(6,6) << goal_point->point.covariance); TODO 
        tracker.push_back(human_coord);
        count++;

        ROS_INFO("Synchronized\n");

        // TODO Drone Safety
    }
};

int main(int argc, char** argv)

{
    ROS_INFO_STREAM  ("Instanciating Formation Node\n");
    ros::init        (argc, argv, "roscpp_formation");
    
    FormationController fc;
    ros::spin();
    return 0;
}