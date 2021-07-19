// Copyright [2021] [Timur Uzakov]
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>


using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

class BlobDetector
{
private:
    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::Publisher human_coord_pub;

    message_filters::Subscriber<Image> image_sub;
    message_filters::Subscriber<Image> depth_sub;
    
    typedef sync_policies::ApproximateTime<Image,Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    
    
    std::string image_sub_topic       = "/uav1/rs_d435/color/image_rect_color";
    std::string depth_sub_topic       = "/uav1/rs_d435/aligned_depth_to_color/image_raw";
    std::string image_pub_topic       = "/uav1/blob_detection";
    std::string human_coord_pub_topic = "/uav1/human/";

    // Output Parameters
    sensor_msgs::ImagePtr                       msg_output;
    geometry_msgs::PoseWithCovarianceStampedPtr msg_human;

    double human_x = 0;
    double human_y = 0;
    double human_z = 0;

    cv::Scalar                  orange_min = cv::Scalar(10,110,110);     //min hsv value orange
    cv::Scalar                  orange_max = cv::Scalar(27,255,255);     //max hsv value orange
    cv::Scalar                  detection_color = cv::Scalar(255,100,0);

    int counter = 0;

public:
    cv::KalmanFilter KF = cv::KalmanFilter(6,3,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(3,1);
    
    boost::array<double, 36UL> msg_cov_array;
    cv::Mat cov_matrix = cv::Mat(6,6,CV_32F, &msg_cov_array);
    
    BlobDetector()
    {
       
        image_sub.subscribe(nh,image_sub_topic,1);
        depth_sub.subscribe(nh,depth_sub_topic,1);

        sync.reset(new Sync(MySyncPolicy(10), image_sub,depth_sub));
        sync->registerCallback(boost::bind(&BlobDetector::callback,this,_1,_2));
        
        image_pub         = nh.advertise<Image>(image_pub_topic, 1);
        human_coord_pub   = nh.advertise<PoseWithCovarianceStamped>(human_coord_pub_topic, 1);

        //---Kalman Filter Parameters---->>----
        KF.transitionMatrix = (cv::Mat_<float>(6,6) <<  1,0,0,1,0,0, 
                                                        0,1,0,0,1,0,
                                                        0,0,1,0,0,1,
                                                        0,0,0,1,0,0,
                                                        0,0,0,0,1,0,
                                                        0,0,0,0,0,1);
        measurement.setTo(cv::Scalar(0));
        KF.statePre.at<float>(0) = 0;
        KF.statePre.at<float>(1) = 0;
        KF.statePre.at<float>(2) = 0;
        KF.statePre.at<float>(3) = 0;
        KF.statePre.at<float>(4) = 0;
        KF.statePre.at<float>(5) = 0;

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(1e-4));
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
        // ---<< Kalman Filter Parameters ----
        ROS_INFO("All functions initialized");
    }
    void PrintThatMessageWasReceived(std::string frame_id)
    {
        
        ROS_INFO_STREAM("[Image from: " <<frame_id<<" ]");
    }
    cv::Point3d PredictUsingKalmanFilter()
    {
        // Prediction, to update the internal statePre variable -->>
        cv::Mat prediction  =   KF.predict();
        cv::Point3d predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
        return  predictPt;
        //  <<---Prediction, to update the internal statePre variable
    }
    cv::Mat ReturnCVMatImageFromMsg(const sensor_msgs::ImageConstPtr& msg)
    {
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
    cv::Mat ReturnCVMatImageFromDepthMsg(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1 );
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            std::cerr<<"Could not convert depth image into CV Mat\n";
        }
        cv::Mat image = cv_ptr->image;
        return  image;
    }
    cv::Mat GaussianBlur(cv::Mat image)
    {
        cv::Mat image_blurred;
        cv::GaussianBlur(image, image_blurred, cv::Size(5,5), 0);
        return  image_blurred;
    }
    cv::Mat BGRtoHSV(cv::Mat image)
    {
        cv::Mat image_HSV;
        cv::cvtColor(image, image_HSV,CV_BGR2HSV);
        return  image_HSV;
    }
    cv::Mat ReturnOrangeMask(cv::Mat image)
    {
        cv::Mat          image_threshold;
        cv::inRange     (image, BlobDetector::orange_min, BlobDetector::orange_max, image_threshold);
        return image_threshold;
    }
    std::vector<std::vector<cv::Point>> ReturnContours(cv::Mat image_threshold)
    {
        std::vector<std::vector<cv::Point>> contours;       //contours are stored here
        std::vector<cv::Vec4i>              hierarchy; 
        cv::findContours(image_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }
    cv::Point2f FindCenter(std::vector<std::vector<cv::Point>> contours, int ID)
    {
        std::vector<cv::Point>  contour_poly   (contours.size());
        cv::Point2f             center         (contours.size());  
        float                   radius         (contours.size());

        cv::approxPolyDP        (contours[ID], contour_poly, 3, true);
        cv::minEnclosingCircle  (contour_poly, center, radius);
        return center;

    }
    float FindRadius(std::vector<std::vector<cv::Point>> contours, int ID)
    {
        std::vector<cv::Point>  contour_poly   (contours.size());
        cv::Point2f             center         (contours.size());  
        float                   radius         (contours.size());

        cv::approxPolyDP        (contours[ID], contour_poly, 3, true);
        cv::minEnclosingCircle  (contour_poly, center, radius);
        return radius;

    }
    void SetMeasurement(cv::Point3d center)
    {
        measurement.at<float>(0) = center.x;
        measurement.at<float>(1) = center.y;
        measurement.at<float>(2) = center.z;
    }
    cv::Point3d UpdateKalmanFilter(cv::Mat_<float>  measurement)
    {
        // Updating Kalman Filter    
        cv::Mat       estimated = KF.correct(measurement);
        cv::Point3d   statePt(estimated.at<float>(0),estimated.at<float>(1),estimated.at<float>(2));
        cv::Point3d   measPt(measurement(0),measurement(1),measurement(2));
        return      statePt;
    }
    void SetGoal(cv::Point3d statePt)
    {
        msg_human->pose.pose.position.x = statePt.x;
        msg_human->pose.pose.position.y = statePt.y; 
        msg_human->pose.pose.position.z = statePt.z;       
    }

     // Function for finding maximal size contour
    int FindMaxAreaContourId(std::vector<std::vector<cv::Point>> contours)
    {
        double  maxArea          = 0;
        int     maxAreaContourId = -1;

        for (size_t i = 0;i<contours.size();i++)
        {
                double   newArea = cv::contourArea(contours.at(i));
                if(newArea > maxArea)
                {
                        maxArea = newArea;
                        maxAreaContourId = i;
                }
        }
        return maxAreaContourId;
    }
    void callback(const ImageConstPtr& msg,const ImageConstPtr& depth_msg)
    {
        ROS_INFO("Synchronized\n");
        std_msgs::Header    msg_header  = msg->header;
        std::string         frame_id    = msg_header.frame_id;
        PrintThatMessageWasReceived (frame_id);
        
        cv::Mat cv_image     = ReturnCVMatImageFromMsg     (msg);
        cv::Mat depth_image  = ReturnCVMatImageFromDepthMsg(depth_msg);
        
        cv::Point3d predictPt= PredictUsingKalmanFilter();
        // -->> Operations on image ----
        // 1) smoothing the image
        cv::Mat     blurred_image   = GaussianBlur(cv_image);
        // 2) conversion to hsv
        cv::Mat     image_HSV       = BGRtoHSV(blurred_image); 
        // 3) finding orange mask
        cv::Mat     image_threshold = ReturnOrangeMask(image_HSV);
        // 4) finding contours
        std::vector<std::vector<cv::Point>> contours = ReturnContours(image_threshold);
        // 5) finding max contour
        int maxAreaContourId        = BlobDetector::FindMaxAreaContourId(contours);
        
        cv::Mat drawing = cv::Mat::zeros(cv_image.size(), CV_8UC3 );
        // 6,7) finding center and kalman filtering
        if (maxAreaContourId>=0)
        {   
            
            cv::Point2f center = FindCenter(contours, maxAreaContourId);
            float       radius = FindRadius(contours, maxAreaContourId);

            cv::Point3d center3D;

            center3D.x = center.x;
            center3D.y = center.y;
            center3D.z = depth_image.at<float>(center.x,center.y)/1000;

            // uncomment the following to draw contours exactly
            /* cv::drawContours(drawing, contours, maxAreaContourId, 
                                detectionColor ,5, cv::LINE_8, hierarchy, 0 );
            */
            // uncomment the following for checking center coordinates
            // std::cout<<center<<'\n';
             //-<<---Blob detector
            // Obtaining the point from Kalman Filter
            SetMeasurement(center3D);
            
            cv::Point3d statePt = UpdateKalmanFilter(measurement);
            
            // uncomment the following for checking estimated center coordinates
            // std::cout<<statePt<<'\n';
            cv::Point2d statePt2D;
            statePt2D.x = statePt.x;
            statePt2D.y = statePt.y;
             // Drawing Point
            cv::circle  (drawing, statePt2D, int(radius), detection_color, 2 );
            cv::circle  (drawing, statePt2D, 5, detection_color, 10);   
            // Set covariance
            cov_matrix = KF.errorCovPost;
            // Setting up goal point
            SetGoal(statePt); 
            
 
            // ROS_INFO_STREAM("[Detected orange object: x "<< statePt.x<<" y "<<statePt.y<<"]"); 

        }
        cv::Mat display = cv_image + drawing;
        msg_output= cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
        msg_output->header.frame_id = std::to_string(counter);
        msg_output->header.stamp = ros::Time::now();
        image_pub.publish(msg_output);

        msg_human->pose.covariance = msg_cov_array;
        msg_human->header.frame_id = std::to_string(counter);
        msg_human->header.stamp = ros::Time::now();
        human_coord_pub.publish(msg_human);
        
        counter++;
    }
};     



int main(int argc, char** argv)
{
    ROS_INFO("BlobDetector node initialized");
    ros::init(argc, argv, "uav1_blob_detector");
    BlobDetector bd;
    ros::spin();

    return 0;
}