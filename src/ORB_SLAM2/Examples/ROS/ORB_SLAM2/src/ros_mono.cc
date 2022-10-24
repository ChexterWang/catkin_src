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

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include <ctime>
#include <ORB_SLAM2/Tracking_status.h>
#include <darknet_ros_msgs/MyBoundingBoxes.h>
#include </home/brian/catkin_ws/devel/include/line_lbd/my_pose.h>
#include </home/brian/catkin_ws/devel/include/line_lbd/My_image.h>
#include </home/brian/catkin_ws/devel/include/line_lbd/my_mat.h>
#include </home/brian/catkin_ws/devel/include/line_lbd/updateServer.h>
// #include "/home/brian/catkin_ws/devel/include/darknet_ros_msgs/MyBoundingBoxes.h"

#define DYSIZE 11
string Dynamic_objects[] = {"person", "bird", "cat", "dog", "horse", "sheep", "cow",
"elephant", "bear", "zebra", "giraffe"};

#define AnchorSize 4

// string Anchor_objects[] = {"laptop", "mouse", "keyboard", "cup", "bottle"};
string Anchor_objects[] = {"mouse", "keyboard", "bottle","tvmonitor"};


using namespace std;

class ImageGrabber
{
public:

    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::Publisher *pub1,ros::Publisher *pub2, ros::Publisher *pub3):
    mpSLAM(pSLAM),pose_pub_(pub1),pose_update_pub_(pub2), pose_tracking_status_pub(pub3){}
    
    void GrabImage(const line_lbd::My_image::ConstPtr& msg);
    bool has_send_initial = false;
    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pose_tracking_status_pub;
    ros::Publisher* pose_pub_;
    ros::Publisher* pose_update_pub_;

    void boxCallback(const darknet_ros_msgs::MyBoundingBoxes::ConstPtr& msg)
    {
        cout << "ORB SLAM get bounding box"<<endl;

        // const std::lock_guard<std::mutex> lock(mutex_image);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image_now, sensor_msgs::image_encodings::RGB8);

        // IRL: will convert to gray image on tracking thread
        cv::Mat img_raw = cv_ptr->image.clone();
        cout << "================"<<stol(msg->header.frame_id, nullptr, 10) << "=============="<<endl; 
        // if(msg->header.frame_id){
        mpSLAM->current_id = stol(msg->header.frame_id, nullptr, 10);
        // }
        
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

            if(is_dynamic(msg->bounding_boxes[i].Class)) dynamic_pos.push_back(tmp);
            if(is_anchor(msg->bounding_boxes[i].Class)) anchor_pos.push_back(tmp);
        
        }
        cout << "orb_slam time: "<< cv_ptr->header.stamp.toSec() << endl;
        // double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif
                // Pass the image to the SLAM system
            // mpSLAM->current_id = stol(msg->id, nullptr, 10);
            cv::Mat Tcw = mpSLAM->TrackMonocular(img_raw,cv_ptr->header.stamp.toSec(),dynamic_pos,anchor_pos);
            
        #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif
/*
        // double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        // times+=ttrack;
        // nums+=1;

        // if(mpSLAM->has_initial && !has_send_initial){
        //     has_send_initial =true;
            

        // }

    // mpSystem->has_initial = true;

    //     for(std::vector<KeyFrame*>::iterator iter= tmp.begin(); iter!=tmp.end();++iter)
    //     {
    //         KeyFrame* kk = *iter;
    //         cout <<"========================keyframe id=================: "<< kk->IRL_id <<endl;
    //         mpSystem->ids.push_back(kk->IRL_id);
    //         mpSystem->poses.push_back(kk->GetPose());
    //     }
*/

        ORB_SLAM2::Tracking_status return_tracking_status;
        return_tracking_status.id = msg->id;
        return_tracking_status.tracking_status = to_string(mpSLAM->GetTrackingState());
        pose_tracking_status_pub->publish(return_tracking_status);

        if(mpSLAM->GetTrackingState()==1) return;
        if(Tcw.rows!=4 && Tcw.cols!=4) return;

        line_lbd::updateServer updated_pose;
            
        // cout << "ORB-SLAM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<mpSLAM->ids.size()<<endl;
        
        for(int i =0;i<3;i++){
            for(int j=0;j<3;j++) updated_pose.rot.push_back(Tcw.at<float>(i,j));
        }

        updated_pose.trans.push_back(Tcw.at<float>(0,3));
        updated_pose.trans.push_back(Tcw.at<float>(1,3));
        updated_pose.trans.push_back(Tcw.at<float>(2,3));

        updated_pose.id = msg->header.frame_id;
        pose_update_pub_->publish(updated_pose);

        line_lbd::my_pose pose;
        pose.id = msg->header.frame_id;
        //TODO test whether transpose

        for(int i =0;i<3;i++){
            for(int j=0;j<3;j++) pose.rot_tcw.push_back(Tcw.at<float>(i,j));
        }

        pose.trans_tcw.push_back(Tcw.at<float>(0,3));
        pose.trans_tcw.push_back(Tcw.at<float>(1,3));
        pose.trans_tcw.push_back(Tcw.at<float>(2,3));

        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mRwc;
        cv::Mat mOw;
        cv::Mat Twc;

        mRcw = Tcw.rowRange(0, 3).colRange(0, 3);
        mRwc = mRcw.t();
        mtcw = Tcw.rowRange(0, 3).col(3);
        mOw = -mRcw.t() * mtcw;

        Twc = cv::Mat::eye(4, 4, CV_32F);
        mRwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
        mOw.copyTo(Twc.rowRange(0, 3).col(3));

        cout << Twc.size()<<endl;
        cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
        cv::Mat twc = Twc.rowRange(0,3).col(3);
        for(int i =0;i<3;i++){
            for(int j=0;j<3;j++) pose.Rotation.push_back(Rwc.at<float>(i,j));
        }

        pose.Trans.push_back(twc.at<float>(0));
        pose.Trans.push_back(twc.at<float>(1));
        pose.Trans.push_back(twc.at<float>(2));
        cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@publish pose"<<endl;
        if(mpSLAM->GetTrackingState()==2){
            pose_pub_->publish(pose);
        }
        if(mpSLAM->GetTrackingState()==3){
            pose.id = "reloc";
            pose_pub_->publish(pose);
        }
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
    
    // double getAverage(){
    //     return times/nums;
    // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    ros::NodeHandle nh;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    ros::Publisher pose_pub = nh.advertise<line_lbd::my_pose>("/SLAM_pose",10);
    ros::Publisher pose_update_pub = nh.advertise<line_lbd::updateServer>("/update_pose_python",10);
    ros::Publisher pose_tracking_status_pub = nh.advertise<ORB_SLAM2::Tracking_status>("/slam_track", 1);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    SLAM.IRL_SLAM = true;
    ImageGrabber igb(&SLAM, &pose_pub, &pose_update_pub, &pose_tracking_status_pub);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/my_bounding_boxes",1, &ImageGrabber::boxCallback, &igb);
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/rgb/My_image", 1, &ImageGrabber::GrabImage,&igb);
    
    ros::spin();
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const line_lbd::My_image::ConstPtr& msg)
{

    //cv_bridge::CvImagePtr cv_ptr; //= cv_bridge::toCvCopy(msg->image_now, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image_now, sensor_msgs::image_encodings::RGB8);

    cv::Mat img_raw = cv_ptr->image.clone();
/*
    // cv::Mat img_raw = cv_ptr->image.clone();
    // Copy the ros image message to cv::Mat.
    // cv_bridge::CvImageConstPtr cv_ptr;
    // try
    // {
    //     //cv_ptr = cv_bridge::toCvShare(msg);
    //     cv_ptr = cv_bridge::toCvCopy(msg->image_now, sensor_msgs::image_encodings::BGR8);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }
*/
    clock_t start,stop;
    start = clock();
    mpSLAM->current_id = stol(msg->id, nullptr, 10);
    cv::Mat Tcw = mpSLAM->TrackMonocular(img_raw,cv_ptr->header.stamp.toSec());
    stop = clock();
    cout << "time of slam for one frame: "<<double(stop-start)/CLOCKS_PER_SEC <<endl;
/*
    // if(mpSLAM->has_initial && !has_send_initial){
    //     has_send_initial =true;
    //     line_lbd::updateServer updated_initial_pose;
    //     // updated_initial_pose.size = mpSLAM->ids.size();
    //     // cout << "ORB-SLAM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<mpSLAM->ids.size()<<endl;
    //     // for(int i =0; i<mpSLAM->ids.size();++i)
    //     // {
    //     //     line_lbd::my_mat tmp;

    //     //     for(int row=0;row<4;row++)
    //     //     {
    //     //         for(int column=0;column<4;column++)
    //     //         {
    //     //             cv::Mat mat = mpSLAM->poses[i];
    //     //             tmp.pose.push_back(mat.at<float>(row,column));
    //     //             // cout << "ORB-SLAM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<mat.at<float>(row,column)<<endl;
    //     //         }
    //     //     }

    //     //     // cout << "ORB-SLAM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<mpSLAM->ids[i]<<endl;
    //     //     updated_initial_pose.ids.push_back(mpSLAM->ids[i]);
    //     //     updated_initial_pose.poses.push_back(tmp);
            
    //     // }

    //     updated_initial_pose.id.push_back(mpSLAM->ids[i]);
    //     pose_update_pub_->publish(updated_initial_pose);

    // }

    // mpSystem->has_initial = true;

    //     for(std::vector<KeyFrame*>::iterator iter= tmp.begin(); iter!=tmp.end();++iter)
    //     {
    //         KeyFrame* kk = *iter;
    //         cout <<"========================keyframe id=================: "<< kk->IRL_id <<endl;
    //         mpSystem->ids.push_back(kk->IRL_id);
    //         mpSystem->poses.push_back(kk->GetPose());
    //     }
*/

    if(Tcw.rows!=4 && Tcw.cols!=4) return; 
    line_lbd::my_pose pose;
    pose.id = msg->id;
    //TODO test whether transpose
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw;
    cv::Mat Twc;

    mRcw = Tcw.rowRange(0, 3).colRange(0, 3);
    mRwc = mRcw.t();
    mtcw = Tcw.rowRange(0, 3).col(3);
    mOw = -mRcw.t() * mtcw;

    Twc = cv::Mat::eye(4, 4, CV_32F);
    mRwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
    mOw.copyTo(Twc.rowRange(0, 3).col(3));

    cout << Twc.size()<<endl;
    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
    cv::Mat twc = Twc.rowRange(0,3).col(3);
    for(int i =0;i<3;i++){
        for(int j=0;j<3;j++)
        {
            pose.Rotation.push_back(Rwc.at<float>(i,j));
        }
    }


    
    pose.Trans.push_back(twc.at<float>(0));
    pose.Trans.push_back(twc.at<float>(1));
    pose.Trans.push_back(twc.at<float>(2));
    cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ publish pose"<<endl;
    pose_pub_->publish(pose);
   

    
    // cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    // cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    // vector<float> q = Converter::toQuaternion(Rwc);

    // f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
}

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


// #include<iostream>
// #include<algorithm>
// #include<fstream>
// #include<chrono>

// #include<ros/ros.h>
// #include <cv_bridge/cv_bridge.h>

// #include<opencv2/core/core.hpp>

// #include"../../../include/System.h"

// using namespace std;

// class ImageGrabber
// {
// public:
//     ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

//     void GrabImage(const sensor_msgs::ImageConstPtr& msg);

//     ORB_SLAM2::System* mpSLAM;
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "Mono");
//     ros::start();

//     if(argc != 3)
//     {
//         cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
//         ros::shutdown();
//         return 1;
//     }    

//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

//     ImageGrabber igb(&SLAM);

//     ros::NodeHandle nodeHandler;
//     ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

//     ros::spin();

//     // Stop all threads
//     SLAM.Shutdown();

//     // Save camera trajectory
//     SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

//     ros::shutdown();

//     return 0;
// }

// void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
// {
//     // Copy the ros image message to cv::Mat.
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
// }



