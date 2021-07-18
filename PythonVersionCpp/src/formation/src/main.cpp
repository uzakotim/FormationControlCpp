#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>


using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class Formation
{
private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Publisher error_pub;

    message_filters::Subscriber<PointStamped> sens_fuse_sub;
    message_filters::Subscriber<Odometry> pose_sub;

    typedef sync_policies::ApproximateTime<PointStamped,Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    Formation()
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

};     



int main(int argc, char** argv)
{
    ROS_INFO("Formation node initialized");
    ros::init(argc, argv, "uav1_formation_controller");
    Formation fc;
    ros::spin();

    return 0;
}