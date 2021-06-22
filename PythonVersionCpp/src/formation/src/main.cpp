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

using namespace sensor_msgs;
using namespace message_filters;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

class FormationController 
{
private:

    message_filters::Subscriber<PointStamped> sub_1;
    message_filters::Subscriber<Odometry> sub_2;
    typedef sync_policies::ApproximateTime<PointStamped,Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    FormationController()
    {
        ros::NodeHandle nh;
        sub_1.subscribe(nh,"computations/goal_point",1);
        sub_2.subscribe(nh,"/odometry/odom_main",1);
    
        //  TODO: Ensure that callback is called
        sync.reset              (new Sync(MySyncPolicy(10), sub_1, sub_2));
        sync->registerCallback  (boost::bind(&FormationController::callback, this, _1, _2));
        ROS_INFO("All functions initialized");
    }
    void callback(const PointStampedConstPtr& goal_point, const nav_msgs::OdometryPtr &pose)
    {
        ROS_INFO("Synchronized\n");
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