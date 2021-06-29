// Copyright [2021] [Timur Uzakov]

#include <iostream>
#include <cmath>
// Time Synchronizer Libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Libraries for msgs
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// CvBridge, Image Transport
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV libraries
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

// Namespaces
using namespace geometry_msgs;
using namespace message_filters;
using namespace sensor_msgs;

class SLAM
{               /*--------------VISUAL SLAM NODE-----------------*/
private:
    ros::NodeHandle                 nh;
    ros::Publisher                  pub;
    ros::Publisher                  pub_point;
    // Messages

    sensor_msgs::ImagePtr           msg_output;
    PointStamped                    msg_goal_point;

    
    // Time Synchronizer variables ---------
    message_filters::Subscriber<PointStamped>   sub_1;
    message_filters::Subscriber<Image>          sub_2;
    typedef sync_policies::ApproximateTime<PointStamped,Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync>                     sync;
    // Time Synchronizer variables --------- 

    // Msgs Topics -------------------------
    std::string                     sub_goal_topic      = "/computations/goal_point";
    std::string                     sub_camera_topic    = "/camera/image"; 
    std::string                     pub_image_topic     = "/slam/matches";
    std::string                     pub_point_topic     = "/slam/points/goal";
    // Msgs Topics -------------------------

    // Visual SLAM variables ---------------
    int                             count {0};
    cv::Mat                         image, image_prev, image_matches;
    const int                       MAX_FEATURES        = 500;
    const float                     GOOD_MATCH_PERCENT  = 0.15f;
    bool                            flag_first_photo    = true;

    cv::Mat                         descriptors;
    cv::Mat                         descriptors_prev;
    std::vector<cv::KeyPoint>       keypoints;
    std::vector<cv::KeyPoint>       keypoints_prev;
    
    cv::Ptr<cv::Feature2D>          orb     = cv::ORB::create(MAX_FEATURES);
    cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch>         matches;
    
    
    std::vector<cv::Point>          points_previous;
    std::vector<cv::Point>          points_current;
    cv::Point                       point_previous;
    cv::Point                       point_current;
    
    bool moved                      = true;
    cv::Mat                         E,R,t,mask,H,P;
    
    // Camera Point
    cv::Point3d                     camera_pose;

    // Visual SLAM variables ---------------

public:
    
    cv::Mat                         point2D = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat                         point3D = cv::Mat::zeros(4,1,CV_64F);
    cv::Mat                         point_destination_2D = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat                         point_destination_3D = cv::Mat::zeros(4,1,CV_64F);
    
    
    SLAM()
    {
    //  SLAM Node Constructor --------------
        
        sub_1.subscribe(nh,sub_goal_topic,1);
        sub_2.subscribe(nh,sub_camera_topic,1);
        
        pub                         = nh.advertise<sensor_msgs::Image>(pub_image_topic,1);
        pub_point                   = nh.advertise<PointStamped>(pub_point_topic,1);
    //  SLAM Node Constructor --------------

    // Parameters for Time Synchronizer, two subscribers connected to one SLAM::callback
        sync.reset(new Sync(MySyncPolicy(10), sub_1,sub_2));
        sync->registerCallback(boost::bind(&SLAM::callback,this,_1,_2));
        
        camera_pose.x = 0;
        camera_pose.y = 0;
        camera_pose.z = 0;
        ROS_INFO("SLAM Node Initialized Successfully");

    }
    // Parameters for Time Synchronizer, two subscribers connected to one SLAM::callback

    cv::Mat ReturnCVMatImageFromMsg(const sensor_msgs::ImageConstPtr& msg)
    {
        // Function that converts Image msg into cv::Mat format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            std::cerr<<"Could not convert image into CV Mat\n";
        }
        cv::Mat image = cv_ptr->image;
        return  image;
    }

    cv::Mat CalculateIntrinsicMatrix(int width, int height, int field_of_view)
    {
        // Function that calculates intrinsic matrix parameters and
        // stores them in cv::Mat object
        
        double x {width/2.0};
        double y {height/2.0};
        cv::Mat K;

        // uncomment for debugging purposes
        // std::cout<<"x: "<<x<<"y: "<<y<<'\n'<<"FOV: "<<field_of_view<<'\n';
        
        double field_of_view_in_radians = field_of_view*(M_PI/180);

        double f_x = x/tan(field_of_view_in_radians/2);
        double f_y = y/tan(field_of_view_in_radians/2);

        
        K = (cv::Mat_<float>(3,3) << f_x, 0,x,   0,f_y,y,   0,0,1); 

        return K;
    }
    cv::Point3d Project2DPointTo3D(cv::Mat projection_2D_to_3D_matrix, cv::Point obstacle_point)
    {
        cv::Mat  point2D      = cv::Mat::zeros(3,1,CV_64F);
        cv::Mat  point3D      = cv::Mat::zeros(4,1,CV_64F);
        cv::Point3d output; 

        point2D.at<double>(0) = obstacle_point.x;
        point2D.at<double>(1) = obstacle_point.y;
        point2D.at<double>(2) = 1;

        // Projection to 3D
        point3D.at<double>(0) = 0;
        point3D.at<double>(1) = 0;
        point3D.at<double>(2) = 0;
        point3D.at<double>(3) = 1;
                    
        point3D = projection_2D_to_3D_matrix*point2D;
        //Normalization
        output.x = point3D.at<double>(0)/point3D.at<double>(3);
        output.y = point3D.at<double>(1)/point3D.at<double>(3);
        output.z = point3D.at<double>(2)/point3D.at<double>(3);
        
        return output;
    }
    void callback(const PointStampedConstPtr& goal_point,const ImageConstPtr& msg)
    {
        // Callback of SLAM Node

        ROS_INFO("[Synchronized and SLAM started]\n");

        moved= true;    // Parameter for SLAM computations

        cv::Mat image  = ReturnCVMatImageFromMsg (msg);

        if (flag_first_photo == true) 
        {   
            // If the frame is taken for the first time, 
            // compute keypoints and descriptors using ORB,
            // clone image frame for output,
            // set the flag to false
            orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            
            image_matches           = image.clone();
            flag_first_photo        = false;

            ROS_INFO("First Photo\n");
            
        }
        else
        {  
            // If the frame is not first, compute current frame,keypoints and descriptors,
            // match with previously stored keypoints and descriptors,
            // sort and draw them
            
            orb     ->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            matcher ->match(descriptors_prev, descriptors, matches, cv::Mat());
            
            std::sort(matches.begin(), matches.end()); //sort matches by score
            const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
            matches.erase(matches.begin()+numGoodMatches,matches.end()); //remove bad matches

            cv::drawMatches(image_prev, keypoints_prev, image,keypoints, matches, image_matches);
        
            // Obtaining all points

            for (size_t i=0; i<matches.size();i++)
            {
                int index_prev {matches.at(i).queryIdx};
                int index_cur  {matches.at(i).trainIdx};

                point_previous = keypoints_prev.at(index_prev).pt;
                point_current  = keypoints.at(index_cur).pt;

                points_previous.push_back(point_previous);
                points_current.push_back(point_current);

            }

            cv::Mat projection_2D_to_3D_matrix = (cv::Mat_<float>(4,3) << 1,0,0,   0,1,0,  0,0,1, 0,0,0);
            
            if (moved == true)
            {   
                // Calculate Matrices If the frame has moved a little bit,
                // i.e. features' positions have changed

                cv::Mat K = CalculateIntrinsicMatrix(image.size[0],image.size[1], 78);
                E = cv::findEssentialMat(points_current,points_previous,K,cv::RANSAC,0.999,1.0,mask);
                cv::recoverPose(E, points_current, points_previous, K, R, t,mask);

                // Calculate camera position
                camera_pose.x = t.at<double>(0);
                camera_pose.y = t.at<double>(1);
                camera_pose.z = t.at<double>(2);  

                ROS_INFO("[Camera Position]");
                std::cout<<camera_pose<<'\n';

                
                cv::hconcat(R,t,H); //Movement Matrix H
                P = (cv::Mat_<float>(3,4) << 0,0,0,0,   0,0,0,0,  0,0,0,0); 
                K.convertTo(K, CV_64F);
                H.convertTo(H, CV_64F);
                P = K*H; // Projection Matrix
                
                cv::Mat P_transp = P.t();
                // ROS_INFO("[Projection Tansposed Matrix]");
                
                // P*X = x

                // P_transp*P*X = P_transp*x
                // X = Inv(P_transp*P)*P_transp*x

                cv::Mat P_transp_P = P_transp*P;
                projection_2D_to_3D_matrix = P_transp_P.inv()*P_transp;
                
            } 
            // PROJECTION OF ALL POINTS
            std::vector<cv::Point3d> obstacles_positions;
            for (size_t i=0; i<points_current.size();i++)
            {
                cv::Point3d projected_obstacle_point = Project2DPointTo3D(projection_2D_to_3D_matrix,points_current[i]);
                obstacles_positions.push_back(projected_obstacle_point); 
            }

            cv::Point   goal_2D;
            goal_2D.x = goal_point->point.x;
            goal_2D.y = goal_point->point.y;
            
            cv::Point3d goal_3D = Project2DPointTo3D(projection_2D_to_3D_matrix,goal_2D);


            ROS_INFO("[Goal Projection to 3D]");
            msg_goal_point.point.x = goal_3D.x;          
            msg_goal_point.point.y = goal_3D.y;
            msg_goal_point.point.z = goal_3D.z;
            msg_goal_point.header.stamp= ros::Time::now();
            msg_goal_point.header.frame_id = std::to_string(count);
            pub_point.publish(msg_goal_point);
            std::cout<<goal_3D<<'\n';
         
        }
        // ****************************** 
        // Preparation of output msg
        msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matches).toImageMsg();
        msg_output->header.frame_id = std::to_string(count);
        msg_output->header.stamp= ros::Time::now();

        // Publishing
        pub.publish(msg_output);
       
        // frame update for ORB detection of features
        image_prev          = image.clone();
        keypoints_prev      = keypoints;
        descriptors_prev    = descriptors;
        count++;
    }


};     



int main(int argc, char** argv)
{
    ROS_INFO("VisualSLAM node initialized");
    ros::init(argc, argv, "vision_node");
    SLAM vs;
    ros::spin();

    return 0;
}