/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "/home/brian/catkin_ws/devel/include/darknet_ros_msgs/MyBoundingBoxes.h"

#include<opencv2/core/core.hpp>
#include<chrono>

#include"../../../include/System.h"

using namespace std;

// #define DYSIZE 11
// string Dynamic_objects[] = {"person", "bird", "cat", "dog", "horse", "sheep", "cow",
// "elephant", "bear", "zebra", "giraffe"};

// #define AnchorSize 0

// // string Anchor_objects[] = {"laptop", "mouse", "keyboard", "cup", "bottle"};
// string Anchor_objects[] = {"mouse", "keyboard", "bottle","tvmonitor"};


class ImageGrabber
{
    public:

    int nums = 0;
    double times = 0;
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){
       
    }

    void boxCallback(const darknet_ros_msgs::MyBoundingBoxes::ConstPtr& msg)
    {
        cout << "ORB SLAM get bounding box"<<endl;

        
        // const std::lock_guard<std::mutex> lock(mutex_image);

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image_now, sensor_msgs::image_encodings::RGB8);

        // IRL: will convert to gray image on tracking thread
        cv::Mat img_raw = cv_ptr->image.clone();
        // cv::Mat img_raw =st.top();
        // st.pop();
        // cv::imshow( "display", img_raw );
        // cv::waitKey(1);
    
        int len = msg->count;

        std::vector<std::vector<int>> anchor_pos;
        std::vector<std::vector<int>> dynamic_pos;


        for(int i=0;i<len;++i)
        {   
            std::vector<int> tmp;
            tmp.push_back(msg->bounding_boxes[i].xmin);
            tmp.push_back(msg->bounding_boxes[i].xmax);
            tmp.push_back(msg->bounding_boxes[i].ymin);
            tmp.push_back(msg->bounding_boxes[i].ymax);

            if(is_dynamic(msg->bounding_boxes[i].Class)){
                dynamic_pos.push_back(tmp);

            }
            
            if(is_anchor(msg->bounding_boxes[i].Class)){
                anchor_pos.push_back(tmp);
            }
        
        }
        cout << "orb_slam time: "<< cv_ptr->header.stamp.toSec() << endl;
        // double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif
                // Pass the image to the SLAM system
                mpSLAM->TrackMonocular(img_raw,cv_ptr->header.stamp.toSec(),dynamic_pos,anchor_pos);
            
        #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        times+=ttrack;
        nums+=1;

    }

    bool is_dynamic(const string Class)
    {
        for(int i=0; i<DYSIZE;++i)
        {
            if(Class.compare(Dynamic_objects[i])==0) return true;
        }
        return false;
    }


    bool is_anchor(const string Class)
    {
        for(int i=0; i<AnchorSize;++i)
        {
            if(Class.compare(Anchor_objects[i])==0) return true;
        }
        return false;
    }
    
    double getAverage(){
        return times/nums;
    }


    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    // ros::NodeHandle nh;

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }
    
    float late = atof(argv[4]);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    SLAM.latency = late;
    SLAM.experiment = true;
    ImageGrabber igb(&SLAM);

    char save_name[100];
    int age = 23;

    // print "My age is " and age variable to buffer variable
    sprintf(save_name, "%s_%s.txt", argv[3],argv[4]);

    cout << "save name: "<<save_name<<endl;

    ros::NodeHandle nodeHandler;
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nodeHandler.subscribe("/my_bounding_boxes",1, &ImageGrabber::boxCallback, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(save_name);
    cout << "total offload: "<<SLAM.offload_num<<endl;
    cout << "average tracking time: "<<igb.getAverage() <<endl;
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}