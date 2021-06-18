#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

        
void callback(const PointStampedConstPtr& image,const ImageConstPtr& image2)
{
    ROS_INFO("Synchronized\n");
    std::cout<<"hello\n";
}


int main(int argc, char** argv)
{
    ROS_INFO("VisualSLAM node initialized");
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle  nh;
    message_filters::Subscriber<PointStamped> sub_1(nh, "/computations/goal_point", 10);
    message_filters::Subscriber<Image> sub_2(nh, "/camera/image",  100); 
    typedef sync_policies::ApproximateTime<PointStamped,Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), sub_1, sub_2); 
    sync.registerCallback(boost::bind(callback,_1,_2));
    ROS_INFO("All functions initialized");
    ros::spin();

    return 0;
}