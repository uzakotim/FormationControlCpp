#include <cmath>
// include message filters and time sync
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
// include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// include ros library
#include <ros/ros.h>


using namespace geometry_msgs;
using namespace message_filters;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace std_msgs;

class FormationController 
{
private:

    message_filters::Subscriber             <PoseWithCovarianceStamped>      sub_1;
    message_filters::Subscriber             <Odometry>                       sub_2;
    typedef sync_policies::ApproximateTime  <PoseWithCovarianceStamped,Odometry>   MySyncPolicy;
    typedef Synchronizer                    <MySyncPolicy>                    Sync;
    boost::shared_ptr<Sync> sync;

// parameters
    double n_pos        = 1.2;
    double n_neg        = 0.5;
    double delta_max    = 0.5;
    double delta_min    = 0.000001;
    double radius       = 6;

    ros::Publisher  pose_pub;
    std::string     pose_topic  = "/uav1/control_manager/goto";

    PointStamped    msg;
    
    cv::Mat                 tracker_vector = (cv::Mat_<float>(3,1) << 0,0,0);
    std::vector<cv::Mat>    tracker;
    
    cv::Mat w_prev = (cv::Mat_<float>(3,1) <<  0,0,0);
    cv::Mat w;
    cv::Mat master_pose;

// measurements
    cv::Mat state,state_cov,human_coord,human_cov;
    int count {0};

public:

    double cost_prev_x{0},cost_cur_x{0};
    double cost_prev_y{0},cost_cur_y{0};
    double cost_prev_z{0},cost_cur_z{0};
    double cost_dif_x{0};
    double cost_dif_y{0};
    double cost_dif_z{0};
    double step_x{0};
    double step_y{0};
    double step_z{0};


    std::vector<double> cost_cur;
    std::vector<double> cost_prev;
    std::vector<double> grad_cur;
    std::vector<double> grad_prev;

    std::vector<double> delta {0.5,0.5,0.5};
    std::vector<double> delta_prev {0.5,0.5,0.5};
    int k{0};  //computing steps
    
    
    double CostX(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat human_cov,double radius)
    {
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(0) - x_prev.at<float>(0)),2) + std::pow((x.at<float>(0) - master_pose.at<float>(0)+radius),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(human_cov)*10e-4;  
        return resulting_cost;
    }
    double CostY(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat human_cov,double radius)
    {
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(1) - x_prev.at<float>(1)),2) + std::pow((x.at<float>(1) - master_pose.at<float>(1)),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(human_cov)*10e-4;  
        return resulting_cost;
    }
    double CostZ(cv::Mat x,cv::Mat x_prev, cv::Mat master_pose,cv::Mat state_cov, cv::Mat human_cov,double radius)
    {
        double offset_z {0.5};
        double resulting_cost{0};
        resulting_cost = std::pow((x.at<float>(2) - x_prev.at<float>(2)),2) + std::pow((x.at<float>(2) - master_pose.at<float>(0)-offset_z),2) + 0.5*cv::determinant(state_cov)*10e-15 + 0.5*cv::determinant(human_cov)*10e-4;  
        return resulting_cost;
    }
    int sign(double x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }

    cv::Mat TrackMasterPose(int count,cv::Mat human_coord,std::vector<cv::Mat> tracker)
    {
        tracker.push_back(human_coord);
        cv::Mat averaged_position = (cv::Mat_<float>(3,1) << 0, 0, 0);
        count++;
        if (count == 2)
        {
            tracker.pop_back();
            tracker.pop_back();
        }
        if (count>22)
        {    
            double sum_x{0},sum_y{0},sum_z{0};

            for (int i =0; i<10;i++)
            {   
                sum_x += tracker[i].at<float>(0);
                sum_y += tracker[i].at<float>(1);
                sum_z += tracker[i].at<float>(2);

                tracker.pop_back();
                count--;
                
            }
            cv::Mat averaged_position = (cv::Mat_<float>(3,1) << sum_x/10.0, sum_y/10.0, sum_z/10.0);
        }
       
        return averaged_position;
  
    }

        

    FormationController()
    {
        ros::NodeHandle nh;
        sub_1.subscribe(nh,"/uav1/sensor_fusion",1);
        sub_2.subscribe(nh,"/uav1/odometry/odom_main",1);
    
//  TODO: Ensure that callback is called
        sync.reset              (new Sync(MySyncPolicy(10), sub_1, sub_2));
        sync->registerCallback  (boost::bind(&FormationController::callback, this, _1, _2));
        
        
// TODO Ensure the sensor fusion message is transported and received
        pose_pub = nh.advertise<PointStamped>(pose_topic, 1);

// measurement
        tracker.push_back(tracker_vector);
        tracker.push_back(tracker_vector);
        ROS_INFO("All functions initialized");


        
    }
    void InitializeCosts()
    {
            cost_prev_x = CostX(w_prev,w_prev,master_pose,state_cov,human_cov,radius);
            cost_prev_y = CostY(w_prev,w_prev,master_pose,state_cov,human_cov,radius);
            cost_prev_z = CostZ(w_prev,w_prev,master_pose,state_cov,human_cov,radius);

// TODO DRY

            cost_cur_x = CostX(w,w_prev,master_pose,state_cov,human_cov,radius);
            cost_cur_y = CostY(w,w_prev,master_pose,state_cov,human_cov,radius);
            cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,human_cov,radius);

            cost_cur.push_back(cost_cur_x);
            cost_cur.push_back(cost_cur_y);
            cost_cur.push_back(cost_cur_z);

            cost_prev.push_back(cost_prev_x);
            cost_prev.push_back(cost_prev_y);
            cost_prev.push_back(cost_prev_z);

            cost_dif_x = (cost_cur_x - cost_prev_x);
            cost_dif_y = (cost_cur_y - cost_prev_y);
            cost_dif_z = (cost_cur_z - cost_prev_z);


            step_x = w.at<float>(0) - w_prev.at<float>(0);
            step_y = w.at<float>(1) - w_prev.at<float>(1);
            step_z = w.at<float>(2) - w_prev.at<float>(2);

            grad_prev.push_back(cost_dif_x/step_x);
            grad_prev.push_back(cost_dif_y/step_y);
            grad_prev.push_back(cost_dif_z/step_z);

    }
    void ComputeGradient()
    {
        cost_cur_x = CostX(w,w_prev,master_pose,state_cov,human_cov,radius);
        cost_cur_y = CostY(w,w_prev,master_pose,state_cov,human_cov,radius);
        cost_cur_z = CostZ(w,w_prev,master_pose,state_cov,human_cov,radius);

        cost_cur[0] = cost_cur_x;
        cost_cur[1] = cost_cur_y;
        cost_cur[2] = cost_cur_z;

        cost_dif_x = (cost_cur_x - cost_prev_x);
        cost_dif_y = (cost_cur_y - cost_prev_y);
        cost_dif_z = (cost_cur_z - cost_prev_z);

        step_x = w.at<float>(0) - w_prev.at<float>(0);
        step_y = w.at<float>(1) - w_prev.at<float>(1);
        step_z = w.at<float>(2) - w_prev.at<float>(2);

        grad_cur[0] = cost_dif_x/step_x;
        grad_cur[1] = cost_dif_y/step_y;
        grad_cur[2] = cost_dif_z/step_z;

    }
    void UpdateCosts(){
        cost_prev_x = cost_cur_x;
        cost_prev_y = cost_cur_y;
        cost_prev_z = cost_cur_z;

        cost_prev[0] = cost_prev_x;
        cost_prev[1] = cost_prev_y;
        cost_prev[2] = cost_prev_z;
    }
// MAIN CALLBACK
    void callback(const PoseWithCovarianceStampedConstPtr& goal_point, const nav_msgs::OdometryPtr &pose)
    {
       


        // measurements
        state       = (cv::Mat_<double>(4,1) << pose->pose.pose.position.x,pose->pose.pose.position.y,pose->pose.pose.position.z,pose->pose.pose.orientation.z);
        // ROS_INFO("state covariance"<<state_cov<<'\n');
        human_coord = (cv::Mat_<double>(3,1) << goal_point->pose.pose.position.x, goal_point->pose.pose.position.y, goal_point->pose.pose.position.z);
        state_cov = (cv::Mat_<float>(6,6)   <<  pose->pose.covariance[0],   pose->pose.covariance[1],   pose->pose.covariance[2],   pose->pose.covariance[3],   pose->pose.covariance[4],   pose->pose.covariance[5],
                                                pose->pose.covariance[6],   pose->pose.covariance[7],   pose->pose.covariance[8],   pose->pose.covariance[9],   pose->pose.covariance[10],  pose->pose.covariance[11],
                                                pose->pose.covariance[12],  pose->pose.covariance[13],  pose->pose.covariance[14],  pose->pose.covariance[15],  pose->pose.covariance[16],  pose->pose.covariance[17],
                                                pose->pose.covariance[18],  pose->pose.covariance[19],  pose->pose.covariance[20],  pose->pose.covariance[21],  pose->pose.covariance[22],  pose->pose.covariance[23],
                                                pose->pose.covariance[24],  pose->pose.covariance[25],  pose->pose.covariance[26],  pose->pose.covariance[27],  pose->pose.covariance[28],  pose->pose.covariance[29],
                                                pose->pose.covariance[30],  pose->pose.covariance[31],  pose->pose.covariance[32],  pose->pose.covariance[33],  pose->pose.covariance[34],  pose->pose.covariance[35]);
        human_cov = (cv::Mat_<float>(6,6)   <<  goal_point->pose.covariance[0], goal_point->pose.covariance[1], goal_point->pose.covariance[2], goal_point->pose.covariance[3], goal_point->pose.covariance[4], goal_point->pose.covariance[5],
                                                goal_point->pose.covariance[6], goal_point->pose.covariance[7], goal_point->pose.covariance[8], goal_point->pose.covariance[9], goal_point->pose.covariance[10],goal_point->pose.covariance[11],
                                                goal_point->pose.covariance[12],goal_point->pose.covariance[13],goal_point->pose.covariance[14],goal_point->pose.covariance[15],goal_point->pose.covariance[16],goal_point->pose.covariance[17],
                                                goal_point->pose.covariance[18],goal_point->pose.covariance[19],goal_point->pose.covariance[20],goal_point->pose.covariance[21],goal_point->pose.covariance[22],goal_point->pose.covariance[23],
                                                goal_point->pose.covariance[24],goal_point->pose.covariance[25],goal_point->pose.covariance[26],goal_point->pose.covariance[27],goal_point->pose.covariance[28],goal_point->pose.covariance[29],
                                                goal_point->pose.covariance[30],goal_point->pose.covariance[31],goal_point->pose.covariance[32],goal_point->pose.covariance[33],goal_point->pose.covariance[34],goal_point->pose.covariance[35]);

 
//        human_cov   = (cv::Mat_<float>(6,6) << goal_point->pose.covariance);

        // human_cov = (cv::Mat_<float>(6,6)   <<  goal_point[6],goal_point[7],goal_point[8],goal_point[9],goal_point[10],goal_point[11],
        //                                         goal_point[12],goal_point[13],goal_point[14],goal_point[15],goal_point[16],goal_point[17],
        //                                         goal_point[18],goal_point[19],goal_point[20],goal_point[21],goal_point[22],goal_point[23],
        //                                         goal_point[24],goal_point[25],goal_point[26],goal_point[27],goal_point[28],goal_point[29],
        //                                         goal_point[30],goal_point[31],goal_point[32],goal_point[33],goal_point[34],goal_point[35],
        //                                         goal_point[36],goal_point[37],goal_point[38],goal_point[39],goal_point[40],goal_point[41]

        // ); 
       
        // Tracker --->>
        master_pose = TrackMasterPose(count,human_coord,tracker);
        // ----<<Tracker

        w           = (cv::Mat_<float>(3,1) <<  state.at<float>(0),state.at<float>(1),state.at<float>(2));

        // RPROP ------------------------------------------------    
        if (w.empty() == false)
        {
            // Run optimization
            // costs
            InitializeCosts();
            // computing longer when standing
            if ((std::abs(w.at<float>(0) - w_prev.at<float>(0))<0.2) || (std::abs(w.at<float>(1) - w_prev.at<float>(1))<0.2) || (std::abs(w.at<float>(2) - w_prev.at<float>(2))<0.2))
            {
                    k = 200;
            } else  k = 50;

            for(int j=0;j<k;j++)
            {
                // Main RPROP loop
               
                ComputeGradient();
                delta_prev = delta;
                for (int i = 0; i<3;i++)
                {
                    if ((grad_prev[i]*grad_cur[i])>0)
                    {
                        delta[i] = std::min(delta_prev[i]*n_pos,delta_max);
                        w_prev.at<float>(i) = w.at<float>(i);
                        w.at<float>(i) = w.at<float>(i) - sign(grad_cur[i])*delta[i];
                        grad_prev[i] = grad_cur[i]; 
                    } else if ((grad_prev[i]*grad_cur[i])<0)
                    {
                        delta[i] = std::max(delta_prev[i]*n_neg,delta_min);
                        if (cost_cur[i] > cost_prev[i])
                        {
                            w_prev.at<float>(i) = w.at<float>(i);
                            w.at<float>(i) = w.at<float>(i)-sign(grad_prev[i])*delta_prev[i];
                        }
                        grad_prev[i] = 0;
                    } else if ((grad_prev[i]*grad_cur[i])==0)
                    {
                        w_prev.at<float>(i) = w.at<float>(i);
                        w.at<float>(i) = w.at<float>(i) - sign(grad_prev[i])*delta[i];
                        grad_prev[i] = grad_cur[i];
                    }
                }
                UpdateCosts();
            }
            std::cout<<"[ Goal Position " << master_pose<< " ]\n";
            
            msg.header.frame_id = "uav1_local_origin";
            msg.header.stamp = ros::Time::now();
            msg.point.x = w.at<float>(0);
            msg.point.y = w.at<float>(1);
            msg.point.z = w.at<float>(2);
            // yaw
            // msg.position.yaw = round(atan2((master_pose[1]-w[1]),(master_pose[0]-w[0])),2)
            // msg.use_yaw = True

            pose_pub.publish(msg);
        }

        w_prev = (cv::Mat_<float>(3,1) <<  state.at<float>(0),state.at<float>(1),state.at<float>(2));
        // RPROP <<----------------------------------------------
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