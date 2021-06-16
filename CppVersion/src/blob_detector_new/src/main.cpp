// include message filters and time sync
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>


// include CvBridge, Image Transport, Image msg

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

// include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// include ros library
#include <ros/ros.h>

using namespace sensor_msgs;
// using namespace message_filters;
using namespace std_msgs;

class BlobDetector 
{
private:
    // Publishers 
    image_transport::Publisher  pub;
    ros            ::Publisher  pub_point;
    image_transport::Subscriber sub;
    // Blob Detector Parametrs
    cv::Scalar                  orange_min = cv::Scalar(10,150,150);     //min hsv value orange
    cv::Scalar                  orange_max = cv::Scalar(27,255,255);     //max hsv value orange
    cv::Scalar                  detection_color = cv::Scalar(255,100,0);
    
    int                         count = 0;
    
    // Output Parameters
    sensor_msgs::ImagePtr       msg_output;
    geometry_msgs::Point        goal;

    // Time Sych Parameters
    // typedef sync_policies::ApproximateTime  <Image, Image> MySyncPolicy;
    // typedef Synchronizer                    <MySyncPolicy> Sync;
    // boost::shared_ptr<Sync>     sync;
public:
    cv::KalmanFilter KF = cv::KalmanFilter(4,2,0);
    cv::Mat_<float>  measurement = cv::Mat_<float>(2,1);
    BlobDetector(ros::NodeHandle *nh)
    {   
        image_transport::ImageTransport it(*nh);
        pub = it.advertise("camera/blob", 1);
        sub = it.subscribe("camera/image", 1, &BlobDetector::image_callback,this);
       
    //  TODO: Ensure that callback is called
        // sync.reset              (new Sync(MySyncPolicy(10), image_sub, depth_sub));
        // sync->registerCallback  (boost::bind(&BlobDetector::image_callback, this, _1, _2));

    //  Detection publisher -->
        pub_point   = nh->advertise<geometry_msgs::Point>("computations/goal_point", 10);
    //   < -- Center point publisher
        

    //---Kalman Filter Parameters---->>----
        KF.transitionMatrix = (cv::Mat_<float>(4,4) << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1);
        measurement.setTo(cv::Scalar(0));
        KF.statePre.at<float>(0) = 0;
        KF.statePre.at<float>(1) = 0;
        KF.statePre.at<float>(2) = 0;
        KF.statePre.at<float>(3) = 0;
        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov,     cv::Scalar::all(1e-4));
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
        setIdentity(KF.errorCovPost,        cv::Scalar::all(.1));
    // ---<< Kalman Filter Parameters ----

    }
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

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        std_msgs::Header    msg_header = msg->header;
        std::string         frame_id = msg_header.frame_id;
        ROS_INFO_STREAM("[Image from: " << frame_id<<" ]");

        // Prediction, to update the internal statePre variable -->>
        cv::Mat prediction  =   KF.predict();
        cv::Point               predictPt(prediction.at<float>(0),prediction.at<float>(1));
        //  <<---Prediction, to update the internal statePre variable

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
        
        
        cv::Mat image_HSV;
        cv::cvtColor(image, image_HSV,CV_BGR2HSV);

        // -->> Operations on image ----
        // 1) gaussian blur
        // cv::Mat          image_blurred;
        // cv::GaussianBlur(image, image_blurred, cv::Size(5,5), 0);
        // 2) conversion to hsv
        // cv::Mat          image_HSV;
        // cv::cvtColor    (image_blurred, image_HSV,CV_BGR2HSV);
        
        // 3) finding orange mask
        cv::Mat          image_threshold;
        cv::inRange     (image_HSV, BlobDetector::orange_min, BlobDetector::orange_max, image_threshold);
        // 4) finding contours
        std::vector<std::vector<cv::Point>> contours;       //contours are stored here
        std::vector<cv::Vec4i>              hierarchy; 
        cv::findContours(image_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        // 5) finding max contour
        int maxAreaContourId    = BlobDetector::FindMaxAreaContourId(contours);
        // 6) finding center and kalman filtering
        cv::Mat drawing         = cv::Mat::zeros(image_threshold.size(), CV_8UC3 );
        
        if (maxAreaContourId>=0)
        {   
            
            std::vector<cv::Point>  contour_poly   (contours.size());
            cv::Point2f             center         (contours.size());  
            float                   radius         (contours.size());

            cv::approxPolyDP        (contours[maxAreaContourId], contour_poly, 3, true);
            cv::minEnclosingCircle  (contour_poly, center, radius);

            // uncomment the following to draw contours exactly
            /* cv::drawContours(drawing, contours, maxAreaContourId, 
                                detectionColor ,5, cv::LINE_8, hierarchy, 0 );
            */
            // uncomment the following for checking center coordinates
            // std::cout<<center<<'\n';

             //-<<---Blob detector
            
            // Obtaining the point fro Kalman Filter
            // std::cout<<"Detected"<<'\n';
            measurement.at<float>(0) = center.x;
            measurement.at<float>(1) = center.y;
            
            // Updating Kalman Filter
            cv::Mat     estimated = KF.correct(measurement);
            cv::Point   statePt(estimated.at<float>(0),estimated.at<float>(1));
            cv::Point   measPt(measurement(0),measurement(1));
            // Drawing Point
            cv::circle  (drawing, statePt, int(radius), detection_color, 2 );
            cv::circle  (drawing, statePt, 5, detection_color, 10);
            // // Setting up goal point
            // goal.x = statePt.x;
            // goal.y = statePt.y;
            // goal.z = image_depth.at<float>(statePt.x,statePt.y)/1000;
            std::cout<<statePt<<'\n';

        }
        cv::Mat display = image + drawing;

        msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
        msg_output->header.frame_id = std::to_string(count);
        count++;
        pub.publish(msg_output);
        ROS_INFO_STREAM("[Image:" << frame_id<<" was sent ]");
    }
};




int main(int argc, char** argv)

{
    ROS_INFO_STREAM  ("Instanciating Blob Detector\n");
    ros::init        (argc, argv, "roscpp_open_cv");
    ros::NodeHandle  nh;
    BlobDetector     bd = BlobDetector(&nh);
    ros::spin();
    return 0;
}
