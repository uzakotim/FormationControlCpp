#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

void callback(const ImageConstPtr& image, const PointStampedConstPtr& point)
{
    ROS_INFO("Synchronized\n");
    std::cout<<"hello\n";
}

int main(int argc, char** argv)
{
    ROS_INFO("VisualSLAM node initialized");
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub(nh, "camera/image", 1);
    message_filters::Subscriber<PointStamped> point_sub(nh, "computations/goal_point", 1);
    
    typedef sync_policies::ApproximateTime<Image, PointStamped> MySyncPolicy;
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),image_sub, point_sub);
    
    sync.registerCallback(boost::bind(&callback,_1,_2));

    ros::spin();

    return 0;
}