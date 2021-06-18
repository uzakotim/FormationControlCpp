// include message filters and time sync
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// include ros library
#include <ros/ros.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std_msgs;
using namespace nav_msgs;

class FormationController 
{
private:
// Time Sych Parameters
    typedef sync_policies::ApproximateTime  <Float32MultiArray, Odometry> MySyncPolicy;
    typedef Synchronizer                    <MySyncPolicy> Sync;
    boost::shared_ptr<Sync>     sync;
public:
    FormationController(ros::NodeHandle *nh)
    {
        message_filters::Subscriber<Float32MultiArray> sens_fuse_sub(*nh,"/uav1/sensor_fusion",1);
        message_filters::Subscriber<Odometry> pose_sub(*nh,"/odometry/odom_main",1);
    
        //  TODO: Ensure that callback is called
        sync.reset              (new Sync(MySyncPolicy(10), sens_fuse_sub, pose_sub));
        sync->registerCallback  (boost::bind(&FormationController::callback, this, _1, _2));

    }
    void callback(const std_msgs::Float32MultiArrayPtr& observation, const nav_msgs::OdometryPtr &pose)
    {
    }
};

int main(int argc, char** argv)

{
    ROS_INFO_STREAM  ("Instanciating Formation Node\n");
    ros::init        (argc, argv, "roscpp_formation");
    ros::NodeHandle     nh;
    FormationController fc = FormationController(&nh);
    ros::spin();
    return 0;
}