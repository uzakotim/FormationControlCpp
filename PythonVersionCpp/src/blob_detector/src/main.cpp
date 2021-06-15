#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roscpp_example");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello from ROS node: " << ros::this_node::getName());
    return 0;

}