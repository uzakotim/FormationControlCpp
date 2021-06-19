#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

class SLAM
{
public:
    SLAM()
    {
       
        sub_1.subscribe(nh,"/computations/goal_point",1);
        sub_2.subscribe(nh,"/camera/image",1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2));
        sync->registerCallback(boost::bind(&SLAM::callback,this,_1,_2));
        ROS_INFO("All functions initialized");
    }
   

    void callback(const PointStampedConstPtr& point,const ImageConstPtr& image2)
    {
        ROS_INFO("Synchronized\n");
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    message_filters::Subscriber<PointStamped> sub_1;
    message_filters::Subscriber<Image> sub_2;
    typedef sync_policies::ApproximateTime<PointStamped,Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
};     



int main(int argc, char** argv)
{
    ROS_INFO("VisualSLAM node initialized");
    ros::init(argc, argv, "vision_node");
    SLAM vs;
    ros::spin();

    return 0;
}