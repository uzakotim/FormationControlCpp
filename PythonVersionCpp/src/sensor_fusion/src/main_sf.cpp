#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class SensFuse
{
public:
    SensFuse()
    {
        sf1_sub.subscribe(nh,sf1_topic,1);
        sf2_sub.subscribe(nh,sf2_topic,1);
        sf3_sub.subscribe(nh,sf3_topic,1);

        human_sub.subscribe (nh,human_topic,1);
        pose_sub.subscribe  (nh,pose_topic,1);
        yaw_sub.subscribe   (nh,yaw_topic,1);
    
        sync.reset(new Sync(MySyncPolicy(10),sf1_sub,sf2_sub,sf3_sub,pose_sub,human_sub,yaw_sub));      
        sync->registerCallback(boost::bind(&SensFuse::callback,this, _1,_2,_3,_4,_5,_6));
        ROS_INFO("All functions initialized");
    
    
    
    }
    void callback()
    {
        ROS_INFO("Synchronized\n");
    }
private:
    ros::NodeHandle nh;
    ros::Publisher human_pub;

    message_filters::Subscriber<PoseWithCovarianceStamped> sf1_sub;
    message_filters::Subscriber<PoseWithCovarianceStamped> sf2_sub;
    message_filters::Subscriber<PoseWithCovarianceStamped> sf3_sub;

    message_filters::Subscriber<Odometry> pose_sub;
    message_filters::Subscriber</*....*/> yaw_sub;
    message_filters::Subscriber<PoseWithCovarianceStamped> human_sub;

    std::string sf1_topic = "/uav1/sensor_fusion";
    std::string sf2_topic = "/uav2/sensor_fusion";
    std::string sf3_topic = "/uav3/sensor_fusion";

    std::string human_topic = "/uav1/human";
    std::string pose_topic  = "/uav1/odometry/odom_main";
    std::string yaw_topic   = "/uav1/compass_yaw";
    
    typedef sync_policies::ApproximateTime<PoseWithCovarianceStamped,PoseWithCovarianceStamped,PoseWithCovarianceStamped,Odometry,PoseWithCovarianceStamped,/*...*/> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
};

int main(int argc, char** argv)
{
    ROS_INFO("Sensor Fusion node initialized");
    ros::init(argc, argv, "fusion_node");
    SensFuse sf;
    ros::spin();

    return 0;
}