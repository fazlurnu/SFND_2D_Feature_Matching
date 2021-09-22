/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

#define LOGGING_KEYPOINTS (false)
#define LOGGING_NB_OF_MATCHES (true)
#define LOGGING_EXECUTION_TIME (false)

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */
    string detectorType = "HARRIS";
    string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
    string binary_or_hog;

    if(argc == 1){
        detectorType = "HARRIS";
        descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
    }
    else if(argc == 2){
        detectorType = argv[1];
    }
    else if(argc == 3){
        detectorType = argv[1];
        descriptorType = argv[2];
    }

    if(descriptorType == "SIFT")
    {
        binary_or_hog = "DES_HOG";
    }
    else{
        binary_or_hog = "DES_BINARY";
    }

    std::cout << detectorType << std::endl;
    std::cout << descriptorType << std::endl;

    #if LOGGING_KEYPOINTS == true
        string filename_keypoints("../log/nb_of_keypoints.csv");
        fstream file_keypoints;
        
        file_keypoints.open(filename_keypoints, std::ios_base::app | std::ios_base::in);
        if (file_keypoints.is_open())
        {
            file_keypoints << detectorType << ",";
        }
    #endif

    #if LOGGING_NB_OF_MATCHES == true
        string filename_matches("../log/nb_of_matches.csv");
        fstream file_matches;
        
        file_matches.open(filename_matches, std::ios_base::app | std::ios_base::in);
        if (file_matches.is_open())
        {
            file_matches << detectorType << "/" << descriptorType << ",";
        }
    #endif

    #if LOGGING_EXECUTION_TIME == true
        string filename_execution_time("../log/execution_time.csv");
        fstream file_execution_time;
        
        file_execution_time.open(filename_execution_time, std::ios_base::app | std::ios_base::in);
        if (file_execution_time.is_open())
        {
            file_execution_time << detectorType << "/" << descriptorType << ",";
        }
    #endif

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    uint32_t total_keypoints = 0;
    uint32_t nb_of_matches = 0;
    double time_detector = 0;
    double time_descriptor = 0;

    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        // keep size of dataBuffer equals to dataBufferSize by erasing the first element
        if (dataBuffer.size() > dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            time_detector += detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if(detectorType.compare("HARRIS") == 0)
        {
            time_detector += detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            time_detector += detKeypointsModern(keypoints, imgGray, detectorType, false);
        }

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = false;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> keypoints_roi;
            for(auto keypoint_iterate = keypoints.begin(); keypoint_iterate<keypoints.end(); keypoint_iterate++)
            {
                if(vehicleRect.contains(keypoint_iterate->pt))
                {
                    cv::KeyPoint tmp_point;
                    tmp_point.pt = cv::Point2f(keypoint_iterate->pt);
                    tmp_point.size = 1;
                    keypoints_roi.push_back(tmp_point);
                }
            }
            keypoints = keypoints_roi;
        }

        total_keypoints += keypoints.size();

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        time_descriptor += descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
            
            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            nb_of_matches += matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                            (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                            matches, binary_or_hog, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done, number of matches " << nb_of_matches << endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }
    } // eof loop over all images

    #if LOGGING_KEYPOINTS == true
        file_keypoints << total_keypoints << std::endl;
    #endif

    #if LOGGING_NB_OF_MATCHES == true
        file_matches << nb_of_matches << std::endl;
    #endif

    #if LOGGING_EXECUTION_TIME == true
        file_execution_time << time_detector + time_descriptor << std::endl;
    #endif

    return 0;
}