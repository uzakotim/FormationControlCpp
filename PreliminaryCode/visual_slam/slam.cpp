#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char** argv)
{

    cv::Mat image, image_prev, image_matches;
    cv::VideoCapture cap;

    // open the default camera using default API
    // cap.open(0);
    // or
    int deviceID = 0;            // 0 = open default camera
    int apiID= cv::CAP_ANY;      // 0 = autodetect default API

    cap.open(deviceID,apiID);
    // check if succeded
    if (!cap.isOpened())
    {
        std::cerr<< "ERROR! Unable to open camera \n";
        return -1;
    } 

    std::cout<<"Start grabbing"<<'\n'<<"Press any key to terminate"<<'\n';
    

    // Visual Slam variables
    const int   MAX_FEATURES        = 500;
    const float GOOD_MATCH_PERCENT  = 0.15f;
    bool        flag_first_photo    = true;

    cv::Mat                     descriptors;
    cv::Mat                     descriptors_prev;
    std::vector<cv::KeyPoint>   keypoints;
    std::vector<cv::KeyPoint>   keypoints_prev;
    
    cv::Ptr<cv::Feature2D>          orb     = cv::ORB::create(MAX_FEATURES);
    cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch>         matches;
    //-------------------------------
   

    for (;;)
    {
        cap.read(image);
        if (image.empty())          // check if succeded
        {
            std::cerr<<"ERROR! blank frame grabbed\n";
            break;
        }

        if (flag_first_photo == true) 
        {   
            // If the frame is taken for the first time, 
            // compute keypoints and descriptors using ORB,
            // clone image frame for output,
            // set the flag to false
            orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            flag_first_photo        = false;
            image_matches           = image.clone();
            std::cout<<"First Photo"<<'\n';
            
        }
        else
        {
            // If the frame is not first, compute current frame
            // keypoints and descriptors, match with previously stored
            // keypoints and descriptors, sort and draw them
            
            orb     ->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
            matcher ->match(descriptors_prev, descriptors, matches, cv::Mat());
            
            std::sort(matches.begin(), matches.end()); //sort matches by score
            const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
            matches.erase(matches.begin()+numGoodMatches,matches.end()); //remove bad matches

            
            cv::drawMatches(image_prev, keypoints_prev, image,keypoints, matches, image_matches);
        }
        
        





        
        cv::namedWindow("Live", cv::WINDOW_AUTOSIZE);
        cv::imshow("Live", image_matches);  // show image
                         //image storage
        if (cv::waitKey(5) >= 0)            // break if a key is pressed
            break;

        // frame update for ORB detection of features
        image_prev          = image.clone();
        keypoints_prev      = keypoints;
        descriptors_prev    = descriptors;
    }
    return 0;
}