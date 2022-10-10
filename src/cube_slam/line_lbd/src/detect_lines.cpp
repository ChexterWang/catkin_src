/*
 * line_detection interface
 * Copyright Shichao Yang,2018, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu
 *
 */

#include <line_lbd/line_descriptor.hpp>
#include <ctime>
#include <unistd.h>


#include<algorithm>

#include<chrono>
// #include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <line_lbd/line_lbd_allclass.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <line_lbd/Keyline_vec.h>
#include <line_lbd/Keyline.h>
#include <line_lbd/My_image.h>

#undef VERBOSE

using namespace cv;
using namespace std;

cv::Mat raw_img;
line_lbd_detect* line_lbd_ptr;
ros::Publisher line_pub;
/*
void imageCallback(const  sensor_msgs::ImageConstPtr& msg)
{
    #ifdef VERBOSE
	  cout << "begin"<<endl;
    #endif
    try{
        raw_img = cv_bridge::toCvShare(msg, "rgb8")->image;

        // done = true;
        // cout << rgb_img.size()<<endl;
        // cout << rgb_img.channels() <<endl;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::vector< KeyLine> keylines_raw,keylines_out;
        line_lbd_ptr->detect_raw_lines(raw_img,keylines_raw);
        line_lbd_ptr->filter_lines(keylines_raw,keylines_out);  // remove short lines
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        #ifdef VERBOSE
	      cout << "time of edge_detect for one frame: "<<ttrack <<endl;
        #endif

        // show image
        // if( raw_img.channels() == 1 )
        //   cvtColor( raw_img, raw_img, COLOR_GRAY2BGR );
        // cv::Mat raw_img_cp;
        // drawKeylines(raw_img, keylines_out, raw_img_cp, cv::Scalar( 0, 150, 0 ),2); // B G R
        // imshow( "Line detector", raw_img_cp );
        // waitKey();
        
        // if (save_to_imgs)
        // {
        //   std::string img_save_name = save_folder+"saved_edges.jpg";
        //   cv::imwrite(img_save_name,raw_img_cp);
        // }
        
        // std::string save_folder = "/home/brian/catkin_ws/src/cube_slam/line_lbd/data/";
        // std::string txt_save_name = save_folder+"saved_edges.txt";
        // ofstream resultsFile;
        // resultsFile.open(txt_save_name);
       
        line_lbd::Keyline_vec key_vec;
        key_vec.id = "test";
        key_vec.length = keylines_out.size();
        for (int j=0;j<keylines_out.size();j++)
        {
          line_lbd::Keyline tmp;
          tmp.startPointX = keylines_out[j].startPointX;
          tmp.startPointY = keylines_out[j].startPointY;
          tmp.endPointX = keylines_out[j].endPointX;
          tmp.endPointY = keylines_out[j].endPointY;
          key_vec.Keylines.push_back(tmp);
          // resultsFile <<keylines_out[j].startPointX <<"\t" <<keylines_out[j].startPointY  <<"\t"
          //         <<keylines_out[j].endPointX   <<"\t" <<keylines_out[j].endPointY    <<endl;
        }
        line_pub.publish(key_vec);
        // resultsFile.close();
        #ifdef VERBOSE
	      cout << "done edge sending"<<endl;
        #endif
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
    
}
*/
void imageCallback(const line_lbd::My_image::ConstPtr & msg)
{
    #ifdef VERBOSE
	  cout << "begin"<<endl;
    #endif
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image_now, sensor_msgs::image_encodings::RGB8);
        raw_img = cv_ptr->image.clone();

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::vector< KeyLine> keylines_raw,keylines_out;
        line_lbd_ptr->detect_raw_lines(raw_img,keylines_raw);
        line_lbd_ptr->filter_lines(keylines_raw,keylines_out);  // remove short lines
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        #ifdef VERBOSE
	      cout << "time of edge_detect for one frame: "<<ttrack <<endl;
        #endif
       
        line_lbd::Keyline_vec key_vec;
        key_vec.id = msg->id;
        key_vec.length = keylines_out.size();
        for (int j=0;j<keylines_out.size();j++)
        {
          line_lbd::Keyline tmp;
          tmp.startPointX = keylines_out[j].startPointX;
          tmp.startPointY = keylines_out[j].startPointY;
          tmp.endPointX = keylines_out[j].endPointX;
          tmp.endPointY = keylines_out[j].endPointY;
          key_vec.Keylines.push_back(tmp);
        }
        line_pub.publish(key_vec);
        #ifdef VERBOSE
	      cout << "done edge sending"<<endl;
        #endif
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert to 'rgb8'.");
    }
    
}

int main( int argc, char** argv )
{
      /* get parameters from comand line */
      // if(argc<2){
      // std::cout<<"Provide an image name"<<endl;
      // return -1;
      // }
      
      int numOfOctave_ = 1;
      float Octave_ratio = 2.0;  

      line_lbd_ptr = new line_lbd_detect(numOfOctave_,Octave_ratio); 
      line_lbd_ptr->use_LSD = true;
      line_lbd_ptr->line_length_thres = 15;  // remove short edges
 
      ros::init(argc, argv, "detect lines");
      ros::NodeHandle nh;
      line_pub = nh.advertise<line_lbd::Keyline_vec>("/edge_detect",10); 
 
      // std::string image_path(argv[1]);
            
      // cv::Mat raw_img = imread( image_path, 1 );
      // if( raw_img.data == NULL )
      // {
      // std::cout << "Error, image could not be loaded. Please, check its path \n"<<image_path << std::endl;
      // return -1;
      // }

      // image_transport::ImageTransport it(nh);
      // image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
      ros::Subscriber sub = nh.subscribe("/camera/rgb/My_image", 1, imageCallback);
      
      ros::spin();
      
      // std::string save_folder;
      // bool use_LSD_algorithm;
      // bool save_to_imgs;
      // bool save_to_txts;
      // nh.param<std::string>("save_folder", save_folder, "$(find line_lbd)/data");
      // nh.param<bool>("use_LSD_algorithm",use_LSD_algorithm,true);
      // nh.param<bool>("save_to_imgs",save_to_imgs,false);
      // nh.param<bool>("save_to_txts",save_to_txts,false);
      
            
      
      
      
      // using my line detector class, could select LSD or edline.
      // cv::Mat out_edges;
      
  
}
