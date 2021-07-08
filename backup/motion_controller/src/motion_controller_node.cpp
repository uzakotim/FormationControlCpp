// include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
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

// Math
#include <math.h>

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
    
    cv::Point3f                 initial_state    = cv::Point3f(0,0,1);
  
public:
    double                      x_previous;
    double                      y_previous;
    double                      z_previous;

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
    double alpha {0.1}; //step parameter
    double gradient;
    double updated;
    
    gradient     = GradientFunction(pose_x, previous,offset);
    updated    = previous - gradient*alpha;
    return  updated;
    }

    double FindGoToPoint(double cost, double pose_x, double previous, double offset)
    {
        // Main loop for finding a go_to point
        // While cost function is greated than threshold, the state will be updating for n steps
        // In case cost function is lower than threshold, the state is preserved
        // pose_x is the coordinate of goal
        double updated;
        int    number_of_steps {100};
        if (std::abs(cost)<=0.01)
        {
            updated = previous;
        }
        else
        {
            for (int j;j<number_of_steps;j++)
            {
                updated       = CalculateUpdate(pose_x,previous,offset);
                
                // uncomment for debugging purposes
                // cost          = CostFunction(pose_x, updated,offset); 
                // ROS_INFO_STREAM("Current Cost:" <<cost<<'\n');      
                
                previous      = updated;
            }
        }
        return updated;

    }
    cv::Mat HumanCoordinateToWorld(cv::Mat image,cv::Mat object_position,double yaw_value,cv::Mat drone_position,cv::Mat offset)
    {
        cv::Mat shift_to_center     = (cv::Mat_<float>(3,1) << image.size[1]/2,image.size[0]/2,0);
        cv::Mat scale_matrix        = (cv::Mat_<float>(3,3) << 0.005,0,0,  0,-0.005,0,     0,0,1); //x same, y flip and rescale
        // uncomment for debugging 
        // std::cout<<shift_to_center<<'\n';
        cv::Mat shifted_and_scaled  = scale_matrix*(object_position - shift_to_center);
        cv::Mat R                   = (cv::Mat_<float>(3,3) << sin(yaw_value),0,cos(yaw_value),    -cos(yaw_value),0,sin(yaw_value) ,   0,1,0);
        cv::Mat rotated_vector      = R*shifted_and_scaled;

        cv::Mat point = drone_position + offset + rotated_vector;
        
        return point;
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
        x_previous = initial_state.x;
        y_previous = initial_state.y;
        z_previous = initial_state.z;
        
        pub = it.advertise(pub_motion_topic, 1);

        ROS_INFO("Motion Controller Node Initialized Successfully"); 
    }
    void callback(const sensor_msgs::PointCloudConstPtr obstacles,const sensor_msgs::ImageConstPtr& msg, const PointStampedConstPtr goal)
    {
        ROS_INFO_STREAM("[ Messages synchronized ]");
        // Receiving and converting image to CV Mat object
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
        //-------------------------------------------------
        //CONVERSION TO WORLD COORDINATES

        //!TODO: Subscribe to YAW and find the yaw value
        // or simulate
        double yaw_value = 0.0;

        //!TODO: Subscribe to Odometry and find the position of robot
        // or simulate
        cv::Mat drone_position  = (cv::Mat_<float>(3,1) << 0,0,0);
        cv::Mat object_position = (cv::Mat_<float>(3,1) << goal->point.x,goal->point.y,goal->point.z);
        cv::Mat offset          = (cv::Mat_<float>(3,1) << (0.2*cos(yaw_value)),(0.2*sin(yaw_value)),0);
        
        cv::Mat object_position_world = HumanCoordinateToWorld(image, object_position,yaw_value, drone_position, offset);
        ROS_INFO_STREAM("[Goal world position]");
        ROS_INFO_STREAM("["<<object_position_world.at<float>(0)<<" | "<<object_position_world.at<float>(1)<<" | "<<object_position_world.at<float>(2)<<"]");
        // -------------------------------------------------

        // GD parameters    
        double x_updated, y_updated, z_updated;
        double current_cost_x, current_cost_y, current_cost_z;
        // Offset determines the position of a robot 
        // relatively to observation
        double offset_x {-2};
        double offset_y {0};
        double offset_z {0};
        
        // Main loop
       
        // Calculation of cost function values
        current_cost_x = CostFunction(object_position_world.at<float>(0), x_previous,offset_x);
        current_cost_y = CostFunction(object_position_world.at<float>(1), y_previous,offset_y);
        current_cost_z = CostFunction(object_position_world.at<float>(2), z_previous,offset_z);
        ROS_INFO_STREAM("[Gradient Descent Computing]"); //Important comment, computer skips gradient sometimes
        // Determining the optimal state
        x_updated = FindGoToPoint(current_cost_x, object_position_world.at<float>(0), x_previous,offset_x);
        y_updated = FindGoToPoint(current_cost_y, object_position_world.at<float>(1), y_previous,offset_y);
        z_updated = FindGoToPoint(current_cost_z, object_position_world.at<float>(2), z_previous,offset_z);

        ROS_INFO_STREAM("[GoTo Destination Position]");        
        ROS_INFO_STREAM("["<<x_updated<<" | "<<y_updated<<" | "<<z_updated<<"]");
        
        x_previous = x_updated;
        y_previous = y_updated;
        z_previous = z_updated;




        msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg_output->header.frame_id = std::to_string(count);
        count++;
        pub.publish(msg_output);
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
