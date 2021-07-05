// include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// include ros library
#include <ros/ros.h>
// Time Sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//  Messages
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>


using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

class MotionController 
{
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<PointCloud>     sub_1;
    message_filters::Subscriber<Image>          sub_2;
    message_filters::Subscriber<PointStamped>   sub_3;
    typedef sync_policies::ApproximateTime<PointCloud,Image,PointStamped> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    image_transport::Publisher  pub;
    int count = 0;
    std::string                 sub_point_cloud_topic = "/ORB/points";
    std::string                 sub_image_topic       = "/camera/image";
    std::string                 sub_goal_topic        = "/camera/goal";

    std::string                 pub_motion_topic      = "/camera/motion";
    
    sensor_msgs::ImagePtr       msg_output;
    
    cv::Scalar                  detection_color = cv::Scalar(255,100,0);
    cv::Scalar                  goal_color = cv::Scalar(50,255,255);

    cv::Point goal_point_on_image;
    cv::Point go_to_point_on_image, go_to_goal;
    
    cv::Point3f                 initial_state    = cv::Point3f(0,0,1);

public:
    double x_previous;
    double y_previous;
    double z_previous;
    
    double CostFunction(double pose_x, double x, double offset)
    {   
    //  Quadratic optimization function
    //  Offset determines a robot's position
    return pow((x-(pose_x + offset)),2);
    }
    double GradientFunction(double pose_x, double x, double offset)
    {
    //  Gradient of the cost function
    //  Offset determines a robot's position
    return 2*(x-(pose_x + offset));
    }

    double CalculateUpdate(double pose_x, double previous,double offset)
    {
    //  Gradient descent update
    //  Offset determines a robot's position
    double alpha {0.01}; //step parameter
    double gradient;
    double updated;
    
    gradient     = GradientFunction(pose_x, previous,offset);
    updated    = previous - gradient*alpha;
    return  updated;
    }

    double FindGoToPoint(double current_cost, double pose_x, double previous, double offset)
    {
        // Main loop for finding a go_to point
        // While cost function is greated than threshold, the state will be updating for n steps
        // In case cost function is lower than threshold, the state is preserved
        double updated;
        int    number_of_steps {50};
        if (std::abs(current_cost)<=0.1)
        {
            updated = previous;
        }
        else
        {
            for (int j;j<number_of_steps;j++)
            {
                updated       = CalculateUpdate(pose_x,previous,offset);
                current_cost    = CostFunction(pose_x, updated,offset);       
                previous      = updated;
            }
        }
        return updated;

    }

    MotionController()
    {    
        sub_1.subscribe(nh,sub_point_cloud_topic,1);
        sub_2.subscribe(nh,sub_image_topic,1);
        sub_3.subscribe(nh,sub_goal_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2,sub_3));
        sync->registerCallback(boost::bind(&MotionController::callback,this,_1,_2,_3));
        image_transport::ImageTransport it(nh);
        
         // Gradient Descent Parameters
        double x_previous = initial_state.x;
        double y_previous = initial_state.y;
        double z_previous = initial_state.z;
        
        pub = it.advertise(pub_motion_topic, 1);
        ROS_INFO("Motion Controller Node Initialized Successfully"); 
    }
    void callback(const sensor_msgs::PointCloudConstPtr obstacles,const sensor_msgs::ImageConstPtr& msg, const PointStampedConstPtr goal)
    {
        std_msgs::Header    msg_header = msg->header;
        std::string         frame_id = msg_header.frame_id;
        // ROS_INFO_STREAM("[Image from: " << frame_id<<" ]");
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;




        goal_point_on_image.x = goal->point.x;
        goal_point_on_image.y = goal->point.y;


       
       
        double x_updated, y_updated, z_updated;
        double current_cost_x, current_cost_y, current_cost_z;
        // Offset determines the position of a robot 
        // relatively to observation
        double offset_x {100};
        double offset_y {0};
        double offset_z {0};

        // Main loop
        // Calculation of cost function values
        current_cost_x = CostFunction(goal_point_on_image.x, x_previous,offset_x);
        current_cost_y = CostFunction(goal_point_on_image.y, y_previous,offset_y);
        current_cost_z = CostFunction(1, z_previous,offset_z);

        // Determining the optimal state
        x_updated = FindGoToPoint(current_cost_x, goal_point_on_image.x, x_previous,offset_x);
        y_updated = FindGoToPoint(current_cost_y, goal_point_on_image.y, y_previous,offset_y);
        z_updated = FindGoToPoint(current_cost_z, 1, z_previous,offset_z);
        
        // std::cout << x_updated<<std::endl;
        // std::cout << y_updated<<std::endl;
       
        std::cout<<"Observed point        "<<" x: "<<goal_point_on_image.x<<" y: "<<goal_point_on_image.y<<" z: "<<1 <<'\n';
        std::cout<<"Robot's go_to position"<<" x: "<<x_updated<<" y: "  <<y_updated<<" z: "  <<z_updated    <<'\n';
        
        go_to_goal.x = x_updated;
        go_to_goal.y = y_updated;

        cv::circle  (image, goal_point_on_image, 5, detection_color, 10);   
        cv::circle  (image, go_to_goal, 5, goal_color, 10);   


        // TODO Measure State
        x_previous = x_updated;
        y_previous = y_updated;
        z_previous = z_updated;




        msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg_output->header.frame_id = std::to_string(count);
        count++;
        pub.publish(msg_output);
        // ROS_INFO_STREAM("[Image:" << frame_id<<" was sent ]");

        ROS_INFO_STREAM("[Motion Controller Synchronized]");
    }
};




int main(int argc, char** argv)

{
    ROS_INFO_STREAM  ("Instanciating Motion Controller\n");
    ros::init        (argc, argv, "roscpp_open_cv");
    ros::NodeHandle  nh;
    MotionController mc;
    ros::spin();
    return 0;
}
