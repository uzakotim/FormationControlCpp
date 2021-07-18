#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Odometry.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

class SensFuse
{
public:
    ros::NodeHandle nh;
    ros::Publisher human_pub;

    message_filters::Subscriber<Odometry> sf1_sub;
    message_filters::Subscriber<Odometry> sf2_sub;
    message_filters::Subscriber<Odometry> sf3_sub;

    message_filters::Subscriber<Odometry> pose_sub;
    message_filters::Subscriber<PointStamped> yaw_sub;
    message_filters::Subscriber<Odometry> human_sub;

    std::string sf1_topic = "/uav1/sensor_fusion";
    std::string sf2_topic = "/uav2/sensor_fusion";
    std::string sf3_topic = "/uav3/sensor_fusion";

    std::string human_topic = "/uav1/human";
    std::string pose_topic  = "/uav1/odometry/odom_main";
    std::string yaw_topic   = "/uav1/compass_yaw";
    
    typedef sync_policies::ApproximateTime<Odometry,Odometry,Odometry,Odometry,Odometry,PointStamped> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    
    int counter = 0;

    // Cov matrix for message
    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);
    
    cv::Mat P_est, x_est, P, cur_state, R, C;
    
    double prev_time_meas {0};
    Odometry msg;

    SensFuse(Odometry msg)
    {
        sf1_sub.subscribe(nh,sf1_topic,1);
        sf2_sub.subscribe(nh,sf2_topic,1);
        sf3_sub.subscribe(nh,sf3_topic,1);

        human_sub.subscribe (nh,human_topic,1);
        pose_sub.subscribe  (nh,pose_topic,1);
        yaw_sub.subscribe   (nh,yaw_topic,1);
    
        sync.reset(new Sync(MySyncPolicy(10),sf1_sub,sf2_sub,sf3_sub,pose_sub,human_sub,yaw_sub));      
        sync->registerCallback(boost::bind(&SensFuse::callback,this, _1,_2,_3,_4,_5,_6));

        human_pub = nh.advertise<Odometry>(sf1_topic,1);
        
        //    init msg
        P_est = cv::Mat::eye(cv::Size(6,6),CV_64F);
        x_est = (cv::Mat_<float>(6,1)<<0,0,0,0,0,0);

        P     = cv::Mat::eye(cv::Size(6,6),CV_64F);
        cur_state = (cv::Mat_<float>(6,1)<<0,0,0,0,0,0);

        // MSG 
        msg.header.frame_id = std::to_string(counter);
        msg.header.stamp = ros::Time::now();
        
        msg.pose.pose.position.x = cur_state.at<float>(0);
        msg.pose.pose.position.y = cur_state.at<float>(1);
        msg.pose.pose.position.z = cur_state.at<float>(2);
        msg.twist.twist.linear.x = cur_state.at<float>(3);
        msg.twist.twist.linear.y = cur_state.at<float>(4);
        msg.twist.twist.linear.z = cur_state.at<float>(5);
        
        cov_matrix = (cv::Mat_<float>(6,6) <<           1,0,0,0,0,0, 
                                                        0,1,0,0,0,0,
                                                        0,0,1,0,0,0,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        
        msg.pose.covariance = msg_cov_array;

        C = (cv::Mat_<float>(3,6) << 1,0,0,0,0,0,
                                     0,1,0,0,0,0,
                                     0,0,1,0,0,0);

        R = (cv::Mat_<float>(6,6) <<                    0.1,0,0,0,0,0, 
                                                        0,0.1,0,0,0,0,
                                                        0,0,0.1,0,0,0,
                                                        0,0,0,0.1,0,0,
                                                        0,0,0,0,0.1,0,
                                                        0,0,0,0,0,0.1);

        ROS_INFO("All functions initialized");
    }

    cv::Mat HumanCoordinateToWorld(cv::Mat object_position,double yaw_value,cv::Mat drone_position,cv::Mat offset_vector)
    {
        cv::Mat shift_to_center     = (cv::Mat_<float>(3,1) << 1280/2,720/2,0); //!TODO! Check image size
        cv::Mat scale_matrix        = (cv::Mat_<float>(3,3) << 0.005,0,0,  0,-0.005,0,     0,0,1); //x same, y flip and rescale
        // uncomment for debugging 
        // std::cout<<shift_to_center<<'\n';
        cv::Mat shifted_and_scaled  = scale_matrix*(object_position - shift_to_center);
        cv::Mat R                   = (cv::Mat_<float>(3,3) << sin(yaw_value),0,cos(yaw_value),    -cos(yaw_value),0,sin(yaw_value) ,   0,1,0);
        cv::Mat rotated_vector      = R*shifted_and_scaled;

        cv::Mat point = drone_position + offset_vector + rotated_vector;
        
        return point;
    }


    void callback(const OdometryConstPtr& sf1,const OdometryConstPtr& sf2,
                  const OdometryConstPtr& sf3, const OdometryConstPtr pose, 
                  const OdometryConstPtr& human, const PointStampedConstPtr yaw)
    {
        ROS_INFO("Synchronized\n");

        // Conversion of state vectors and their covariances into CV::Mat format

        cv::Mat sf1_state = (cv::Mat_<float>(6,1) << sf1->pose.pose.position.x,sf1->pose.pose.position.y,
                                                     sf1->pose.pose.position.z, sf1->twist.twist.linear.x,
                                                     sf1->twist.twist.linear.y,sf1->twist.twist.linear.z);

        cv::Mat sf1_cov   = (cv::Mat_<float>(6,6) << sf1->pose.covariance);

        cv::Mat sf2_state = (cv::Mat_<float>(6,1) << sf2->pose.pose.position.x,sf2->pose.pose.position.y,
                                                     sf2->pose.pose.position.z, sf2->twist.twist.linear.x,
                                                     sf2->twist.twist.linear.y,sf2->twist.twist.linear.z);

        cv::Mat sf2_cov   = (cv::Mat_<float>(6,6) << sf2->pose.covariance);

        cv::Mat sf3_state = (cv::Mat_<float>(6,1) << sf3->pose.pose.position.x,sf3->pose.pose.position.y,
                                                     sf3->pose.pose.position.z, sf3->twist.twist.linear.x,
                                                     sf3->twist.twist.linear.y,sf3->twist.twist.linear.z);

        cv::Mat sf3_cov   = (cv::Mat_<float>(6,6) << sf3->pose.covariance);


        // Measurement

        cv::Mat state  = (cv::Mat_<float>(3,1) << pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z );
        cv::Mat offset = (cv::Mat_<float>(3,1) << (0.2*cos(yaw->point.x)),(0.2*sin(yaw->point.x)),0); //TODO needs working yaw measurement
        cv::Mat human_coord = (cv::Mat_<float>(3,1)<< human->pose.pose.position.x, human->pose.pose.position.y,human->pose.pose.position.z);

        cv::Mat meas_state = HumanCoordinateToWorld(human_coord,yaw->point.x,state,offset);
        cv::Mat meas_cov   = (cv::Mat_<float>(6,6) << human->pose.covariance);

        // KALMAN
        // Correction
        cv::Mat S = C*P_est*C.t() + C*meas_cov*C.t();;
        cv::Mat K = P_est*C.t()*S.inv();
        cv::Mat innovation = meas_state - C*x_est;
        // sf1 output
        cur_state = x_est + K*innovation;
        P = cv::Mat::eye(cv::Size(6,6),CV_64F) - K*C*P_est;
        // publishing cur state
        // MSG 
        msg.header.frame_id = std::to_string(counter);
        msg.header.stamp = ros::Time::now();
        
        msg.pose.pose.position.x = cur_state.at<float>(0);
        msg.pose.pose.position.y = cur_state.at<float>(1);
        msg.pose.pose.position.z = cur_state.at<float>(2);
        msg.twist.twist.linear.x = cur_state.at<float>(3);
        msg.twist.twist.linear.y = cur_state.at<float>(4);
        msg.twist.twist.linear.z = cur_state.at<float>(5);
        
        cov_matrix = P;
        msg.pose.covariance = msg_cov_array;

        double time_meas = human->header.stamp.sec + yaw->header.stamp.nsec/1000000000;
        double time_dif = time_meas - prev_time_meas;

        cv::Mat A = (cv::Mat_<float>(6,6) <<            1,0,0,time_dif,0,0, 
                                                        0,1,0,0,time_dif,0,
                                                        0,0,1,0,0,time_dif,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        // Prediction

        x_est = A*(sf1_state+sf2_state+sf3_state)/3.0;
        P_est = (A*(sf1_cov + sf2_cov + sf3_cov)/3.0)*A.t() + R;
        
        
        prev_time_meas = time_meas;
        counter++;

    }

};

int main(int argc, char** argv)
{
    Odometry msg;
    ROS_INFO("Sensor Fusion node initialized");
    ros::init(argc, argv, "uav1_sensor_fusion");
    SensFuse sf(msg);
    ros::Rate rate(100);
    while (!(ros::isShuttingDown))
    {
        sf.human_pub.publish(msg);
        ros::spin();
    }
    

    return 0;
}