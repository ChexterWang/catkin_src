// std c
#include <stdio.h>
#include <math.h> 
#include <iostream>
#include <string>
#include <mutex>
#include <stack>
#include <map>

// opencv
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include<chrono>
// ros
#include <ros/ros.h>
#include <ros/package.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/MyBoundingBoxes.h"

// ours
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "tictoc_profiler/profiler.hpp"
#include <line_lbd/Keyline_vec.h>
#include <line_lbd/Keyline.h>
#include <line_lbd/my_pose.h>
#include <line_lbd/final_pose.h>
#include <line_lbd/combined_boxes.h>
#include <line_lbd/anchor_objects.h>
#include <line_lbd/anchor_pose.h>

using namespace std;
using namespace Eigen;

/* classes:
    person
    bicycle
    car
    motorbike
    aeroplane
    bus
    train
    truck
    boat
    traffic light
    fire hydrant
    stop sign
    parking meter
    bench

    backpack
    umbrella
    handbag
    tie
    suitcase
    frisbee
    skis
    snowboard
    sports ball
    kite
    baseball bat
    baseball glove
    skateboard
    surfboard
    tennis racket
    bottle
    wine glass
    cup
    fork
    knife
    spoon
    bowl
    banana
    apple
    sandwich
    orange
    broccoli
    carrot
    hot dog
    pizza
    donut
    cake
    chair
    sofa
    pottedplant
    bed
    diningtable
    toilet
    tvmonitor
    laptop
    mouse
    remote
    keyboard
    cell phone
    microwave
    oven
    toaster
    sink
    refrigerator
    book
    clock
    vase
    scissors
    teddy bear
    hair drier
    toothbrush
*/

#define DYSIZE 14
string Dynamic_objects[] = {"person", "bird", "cat", "dog", "horse", "sheep", "cow",
"elephant", "bear", "zebra", "giraffe","diningtable", "tvmonitor","chair"};

#define AnchorSize 2
// string Anchor_objects[] = {"laptop","book", "mouse", "keyboard", "cup", "bottle", "pottedplant"};
string Anchor_objects[] = {"mouse", "bottle"};

#undef VERBOSE

// bool done =false;
class Box_handler{
    private:
        std::mutex mutex_image;
        ros::Subscriber sub;
        ros::Subscriber edge_sub;
        ros::Subscriber box_sub;
        ros::Subscriber pose_sub;
        ros::Publisher anchor_pub;
        ros::Publisher final_pub;
        ros::NodeHandle nh;

        map<string, bool> has_edge; // viewer
        map<string, bool> has_pose; // viewer
        bool has_image = false;
        map<string, bool> initial; // viewer

        map<string, line_lbd::my_pose> current_pose; // viewer
        map<string, line_lbd::my_pose> pose; // viewer
        detect_3d_cuboid detect_cuboid_obj;
        
        map<string, Eigen::MatrixXd> all_lines_raw; // viewer
        Matrix3d Kalib;
/*      // stack<cv::Mat> st;
        // cv::Mat InitToGround; 
        // Matrix4d trans;
        
        //  rgb_img;
*/
    public:
        Box_handler(){
            has_edge["0"], has_pose["0"], initial["0"] = false, false, false;
            all_lines_raw["0"] = Eigen::MatrixXd(100,4);
            image_transport::ImageTransport it(nh);
            final_pub = nh.advertise<line_lbd::final_pose>("/final_pose",1);
            anchor_pub = nh.advertise<line_lbd::anchor_objects>("/anchor_objects",1);
            sub = nh.subscribe("/camera/rgb/image_raw", 1, &Box_handler::imageCallback, this);
            edge_sub = nh.subscribe("/edge_detect",10 ,&Box_handler::edgeCallback,this);
            box_sub = nh.subscribe("/my_bounding_boxes",1,&Box_handler::boxCallback,this);
            pose_sub = nh.subscribe("/SLAM_pose",10,&Box_handler::poseCallback,this);
            Kalib << 737.037, 0, 340.565,
                0, 699.167, 218.486,
                0, 0, 1.0000;
/*          Kalib << 529.5000, 0, 365.0000,
                0, 529.5000, 265.0000,
                0, 0, 1.0000;

            trans << 1, 0, 0, 0,
                0, 1, 0,0,
                0, 0, 1,1,
                0, 0,0, 1;
*/
            detect_cuboid_obj.whether_plot_detail_images = false;
            detect_cuboid_obj.whether_plot_final_images = true;
            detect_cuboid_obj.print_details = false; // false  true
            detect_cuboid_obj.set_calibration(Kalib);
            detect_cuboid_obj.whether_sample_bbox_height = false;
            detect_cuboid_obj.whether_sample_cam_roll_pitch = false;
/*          // set initial camera pose wrt ground. by default camera parallel to ground, height=1.7 (kitti)
            double init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw;
            n.param<double>("init_x", init_x, 0);
            n.param<double>("init_y", init_y, 0);
            n.param<double>("init_z", init_z, 1.7);
            n.param<double>("init_qx", init_qx, -0.7071);
            n.param<double>("init_qy", init_qy, 0);
            n.param<double>("init_qz", init_qz, 0);
            n.param<double>("init_qw", init_qw, 0.7071);
*/
            Eigen::Quaternionf pose_quat(0.7071, -0.7071, 0, 0);
            Eigen::Matrix3f rot = pose_quat.toRotationMatrix(); // 	The quaternion is required to be normalized
/*          InitToGround = cv::Mat::eye(4, 4, CV_32F);
            for (int row = 0; row < 3; row++)
                for (int col = 0; col < 3; col++)
                    InitToGround.at<float>(row, col) = rot(row, col);
            InitToGround.at<float>(0, 3) = 0;
            InitToGround.at<float>(1, 3) = 0;
            InitToGround.at<float>(2, 3) = 1.35;
*/
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

        void imageCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            try{
/*              const std::lock_guard<std::mutex> lock(mutex_image);
                
                cv::Mat rgb_img = cv_bridge::toCvShare(msg, "rgb8")->image;
                st.push(rgb_img);
                done = true;
                cout << rgb_img.size()<<endl;
                cout << rgb_img.channels() <<endl;
*/
                has_image = true;
            }catch(cv_bridge::Exception& e){
                ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
            }
            
        }

        void poseCallback(const line_lbd::my_pose::ConstPtr & msg )
        {
            #ifdef VERBOSE
            cout << "get pose !!!!!!!!!!!!!!!!!"<<endl;
            #endif
            if(has_pose.size() > 50) return;
            string _id = msg->id;
            current_pose[_id].Rotation=msg->Rotation;
            current_pose[_id].Trans=msg->Trans;
            current_pose[_id].rot_tcw = msg->rot_tcw;
            current_pose[_id].trans_tcw = msg->trans_tcw;
            has_pose[_id] = true;
            initial[_id] = true;
        }

        void edgeCallback(const line_lbd::Keyline_vec::ConstPtr& msg)
        {
            #ifdef VERBOSE
            cout << "edge detected!!!"<<endl;
            #endif
            if(has_edge.size() > 50) return;
            string _id = msg->id;
            all_lines_raw[_id] = Eigen::MatrixXd(100,4);
            line_lbd::Keyline tmp;
            int row_counter = 0;
            for(int i=0;i<msg->length;++i)
            {
                tmp = msg->Keylines[i];
                all_lines_raw[_id](i,0) = tmp.startPointX;
                all_lines_raw[_id](i,1) = tmp.startPointY;
                all_lines_raw[_id](i,2) = tmp.endPointX;
                all_lines_raw[_id](i,3) = tmp.endPointY;
                row_counter++;
                if (i >= all_lines_raw[_id].rows()) // if matrix row is not enough, make more space.
                        all_lines_raw[_id].conservativeResize(all_lines_raw[_id].rows() * 2, all_lines_raw[_id].cols());
            }

            all_lines_raw[_id].conservativeResize(row_counter, all_lines_raw[_id].cols());
            has_edge[_id] = true;  
        }

        void boxCallback(const darknet_ros_msgs::MyBoundingBoxes::ConstPtr& msg)
        {
            string _id = msg->id;
            #ifdef VERBOSE
            // If _id can be found in has_pose, then print has_pose[_id]
            cout << "boxCallback has_pose: "<< (has_pose.find(_id) != has_pose.end())?has_pose[_id]:false <<endl;
            #endif

            // Short circuit evaluation: if lhs is true, rhs will not be checked.
            // If _id can be found in has_edge(lhs is false), it will check if has_edge[_id] is true
            // Theoretically, if there is record in has_edge, it should be true
            // Same rules are apply to initial. If neither has_edge nor initial are true, then return
            if(has_edge.find(_id) == has_edge.end() || !has_edge[_id]){
                if(initial.find(_id) == initial.end() || !initial[_id]) return;
            }
            
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image_now, sensor_msgs::image_encodings::RGB8);
            cv::Mat img_raw = cv_ptr->image.clone();
/*
            // const std::lock_guard<std::mutex> lock(mutex_image);

            // cv::Mat img_raw =st.top();
            // st.pop();
            // cv::imshow( "display", img_raw );
            // cv::waitKey(1);
*/
            int len = msg->count;
            int objects_count = 0;

            std::vector<int> static_objects;
            std::vector<bool> anchor_objects;
            std::vector<string> anchor_names;

            for(int i=0;i<len;++i)
            {   
                if(is_dynamic(msg->bounding_boxes[i].Class)) continue;
                if(!is_anchor(msg->bounding_boxes[i].Class)) continue;
/*
                // if(is_anchor(msg->bounding_boxes[i].Class)){
                //     anchor_objects.push_back(true);
                //     anchor_names.push_back(msg->bounding_boxes[i].Class);
                // }else{
                //     anchor_objects.push_back(false);
                // }
*/
                anchor_objects.push_back(false);
                anchor_names.push_back(msg->bounding_boxes[i].Class);
                static_objects.push_back(i);
                objects_count++;
            }
            if(objects_count==0) return;

            // cout << "objects: "<<objects_count<<endl;
            
            MatrixXd obj_bbox_coors(objects_count,5);
            int tmp = 0;
            for(std::vector<int>::iterator it = static_objects.begin(); it != static_objects.end(); ++it)
            {
                // cout << "iter: "<<*it<<endl;
                obj_bbox_coors(tmp,0) = msg->bounding_boxes[*it].xmin; //xmin
                obj_bbox_coors(tmp,1) = msg->bounding_boxes[*it].ymin; //ymin
                obj_bbox_coors(tmp,2) = msg->bounding_boxes[*it].xmax-msg->bounding_boxes[*it].xmin; //w
                obj_bbox_coors(tmp,3) = msg->bounding_boxes[*it].ymax-msg->bounding_boxes[*it].ymin; //h
                obj_bbox_coors(tmp,4) = msg->bounding_boxes[*it].probability;
                tmp++;      
            }
            obj_bbox_coors.leftCols<2>().array() -= 1;
/*
            // Kalib << 737.037, 0, 340.565,
            //     0, 699.167, 218.486,
            //     0, 0, 1.0000;

            // cv::Mat R = pose.Rotation;

            
            // if(!has_pose){
            //     trans << 1, 0, 0, 0,
            //     0, 1, 0, 0,
            //     0, 0, 1, 1,
            //     0, 0, 0, 1; 
    
            // }
*/
            Matrix4f trans_tmp;
            cv::Mat frame_pose_to_ground;
            
            if(has_pose[_id]){
                pose[_id] = current_pose[_id];
                float sy = sqrt(pose[_id].Rotation.at(0) * pose[_id].Rotation.at(0) +  pose[_id].Rotation.at(3) * pose[_id].Rotation.at(3));
                bool singular = sy < 1e-6;
                float x, y, z;
                if (!singular)
                {
                    x = atan2(pose[_id].Rotation.at(7) , pose[_id].Rotation.at(8));
                    y = atan2(-pose[_id].Rotation.at(6), sy);
                    z = atan2(pose[_id].Rotation.at(3), pose[_id].Rotation.at(0));
                }
                else
                {
                    x = atan2(-pose[_id].Rotation.at(5), pose[_id].Rotation.at(4));
                    y = atan2(-pose[_id].Rotation.at(6), sy);
                    z = 0;
                }
/*                
                // cout << "==================================================="<<endl;
                // cout << "x: "<< x <<"y: "<< y <<"z: "<< z << endl;
                // // cout << pose.Rotation.at(0) <<", "<<pose.Rotation.at(1)<<", "<<pose.Rotation.at(2)<<endl;
                // // cout << pose.Rotation.at(3) <<", "<<pose.Rotation.at(4)<<", "<<pose.Rotation.at(5)<<endl;
                // // cout << pose.Rotation.at(6) <<", "<<pose.Rotation.at(7)<<", "<<pose.Rotation.at(8)<<endl;
                // cout << "==================================================="<<endl;
                // transToWolrd << pose.Rotation.at(0), pose.Rotation.at(1), pose.Rotation.at(2), 0, // hard coded  NOTE if accurate camera roll/pitch, could sample it!
                //     pose.Rotation.at(3), pose.Rotation.at(4), pose.Rotation.at(5), 0,
                //     pose.Rotation.at(6), pose.Rotation.at(7), pose.Rotation.at(8), 1.35,
                //     0, 0, 0, 1;
                // x+=0.45;
                
                // trans_tmp << cos(y), 0, sin(y), 0,
                // 0, 1, 0,0,
                // -sin(y), 0, cos(y),1,
                // 0, 0,0,1;
*/
                trans_tmp <<1, 0, 0, 0, 
                0, cos(x), -sin(x),0,
                0, sin(x), cos(x), 1,
                0, 0,0,1;
/*
                // cv::Mat trans;
                // if (build_worldframe_on_ground){ // if initial world frame is on ground, directly use it.
                // trans = frame_pose_to_init;
                // cv::Mat frame_pose_to_init = cv::Mat::eye(4, 4, CV_32F);
                
                // for(int i=0; i<3; i++){
                //     for(int j=0;j<3;++j){
                //         cout << "======================="<<i<<", "<<j<<"============"<<endl;
                //         frame_pose_to_init.at<float>(i, j) = pose.Rotation.at(i*3+j);
                //     }
                // }

                // for(int i=0;i<3;++i){
                //     cout << "=======================Trans"<<i<<"============"<<endl;
                //     frame_pose_to_init.at<float>(i, 3) = pose.Trans.at(i);
                // }

            //     tmp_ << 1, 0.0011, 0.0004, 0, // hard coded  NOTE if accurate camera roll/pitch, could sample it!
            // 0, -0.3376, 0.9413, 0,
            // 0.0011, -0.9413, -0.3376, 1.35,
            // 0, 0, 0, 1;
                // c::Mat tmp_ = cv::Mat::eye(4,4,CV_32F);
                // tmp_.at<float>()

                // frame_pose_to_ground =  InitToGround*frame_pose_to_init;
                
                // trans << pose.Rotation.at(0),pose.Rotation.at(1),pose.Rotation.at(2), 0,
                // pose.Rotation.at(6), pose.Rotation.at(7), pose.Rotation.at(8),0,
                // pose.Rotation.at(3), pose.Rotation.at(4), pose.Rotation.at(5),1.35,
                // 0,0,0,1;
*/
                #ifdef VERBOSE
                cout << "==================================================="<<endl;
                cout << pose[_id].Rotation.at(0) <<", "<<pose[_id].Rotation.at(1)<<", "<<pose[_id].Rotation.at(2)<<endl;
                cout << pose[_id].Rotation.at(3) <<", "<<pose[_id].Rotation.at(4)<<", "<<pose[_id].Rotation.at(5)<<endl;
                cout << pose[_id].Rotation.at(6) <<", "<<pose[_id].Rotation.at(7)<<", "<<pose[_id].Rotation.at(8)<<endl;
                cout << "==================================================="<<endl;
                #endif
/*
                // } else { // if not, apply T_ground_init
                //     frame_pose_to_ground = InitToGround * frame_pose_to_init;
                // }

                // pop_pose_to_ground = frame_pose_to_ground;
                
                // detect_cuboid_obj->detect_cuboid(pKF->raw_img, cam_transToGround.cast<double>(), all_obj2d_bbox_infov_mat, all_lines_raw, all_obj_cubes);

                

                // trans = trans_tmp;
*/
            }

            // has_pose = false;
/*
            // Eigen::Matrix<float, 4, 4> transToWolrd_;

            // for (int ii = 0; ii < 4; ii++)
            //     for (int jj = 0; jj < 4; jj++)
            //         transToWolrd_(ii, jj) = frame_pose_to_ground.at<float>(ii, jj);
*/                    
            Matrix4f transToWolrd;
            Matrix4f tmp_;
            tmp_ << 1, 0.0011, 0.0004, 0, // hard coded  NOTE if accurate camera roll/pitch, could sample it!
            0, -0.3376, 0.9413, 0,
            0.0011, -0.9413, -0.3376, 0.11,
            0, 0, 0, 1;
            transToWolrd = trans_tmp*tmp_;
            
            line_lbd::final_pose return_pose;
            cv_bridge::CvImage cvImage_raw;
            cvImage_raw.header.stamp = ros::Time::now();
            cvImage_raw.header.frame_id = "detection_image";
            cvImage_raw.encoding = sensor_msgs::image_encodings::RGB8;
            cvImage_raw.image = cv::Mat(img_raw.clone()) ;
            return_pose.image_raw = *cvImage_raw.toImageMsg();
            
            std::vector<ObjectSet> all_object_cuboids;
            // // if(done)
            // // {
            #ifdef VERBOSE
            cout << "img_raw: "<<img_raw.size()<<endl;
            cout << "all lines raw: "<<all_lines_raw[_id].rows()<<", "<< all_lines_raw[_id].cols()<<endl;
            #endif
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            detect_cuboid_obj.detect_cuboid(img_raw, transToWolrd.cast<double>(), obj_bbox_coors, all_lines_raw[_id], all_object_cuboids,anchor_objects);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            #ifdef VERBOSE
            cout << "time of bounding_box for one frame: "<<ttrack <<endl;
            #endif

            return_pose.id = msg->id;
            #ifdef VERBOSE
            cout << "pose rotation size: "<<pose[_id].Rotation.size()<<endl;
            #endif
            for(int i=0; i<9; ++i)
            {
                return_pose.rot.push_back(pose[_id].rot_tcw.at(i));
            }

            #ifdef VERBOSE
            cout << "pose Transition size: "<<pose[_id].Trans.size()<<endl;
            #endif
            for(int i=0; i<3; ++i)
            {
                return_pose.trans.push_back(pose[_id].trans_tcw.at(i));
            }

            // return_pose.count = all_object_cuboids.size();
            int count_ = 0;
            int tmp_count = 0;
            
            for(std::vector<ObjectSet>::iterator it = all_object_cuboids.begin();it!=all_object_cuboids.end();++it)
            {
                line_lbd::combined_boxes box;
                std::vector<cuboid *> cuboids = *it;
                if(cuboids.size()==0) {
                    tmp_count++;
                    continue;
                }
                if(anchor_objects[tmp_count]){
                    // cout << "========================================="<<anchor_names[tmp_count]<< "=================================================="<<endl;
                    box.class_name = anchor_names[tmp_count];                 
                }
                cuboid *tmp = cuboids[0];
                // config_type 2*1
                #ifdef VERBOSE
                cout << "box_config_type size: rows: "<<tmp->box_config_type.rows()<<"column: "<<tmp->box_config_type.cols()<<endl;
                #endif
                box.type_.push_back(tmp->box_config_type(0,0));
                box.type_.push_back(tmp->box_config_type(1,0));
                //2d corners 2*8
                
                // cout << "box_corners_2d: rows: "<<tmp->box_corners_2d.rows()<<"column: "<<tmp->box_corners_2d.cols()<<endl;
                for(int row=0;row<2;++row)
                {
                    for(int column=0;column<8;++column)
                    {
                        box.BoundingBox2D.push_back(tmp->box_corners_2d(row,column));
                    }
                }

                // cout << "box_corners_3d: rows: "<<tmp->box_corners_3d_world.rows()<<"column: "<<tmp->box_corners_3d_world.cols()<<endl;
                //3d corners 3*8
                for(int row=0;row<3;++row)
                {
                    for(int column=0;column<8;++column)
                    {
                        box.BoundingBox3D.push_back(tmp->box_corners_3d_world(row,column));
                    }
                }
                return_pose.boxes.push_back(box);
/*
                // if(tmp->box_corners_2d(0,0))
                // {
                //     int right = tmp->box_corners_2d(0,0);
                //     int left = tmp->box_corners_2d(0,0);
                //     int top = tmp->box_corners_2d(1,0);
                //     int bottom = tmp->box_corners_2d(1,0);

                //     if(anchor_objects[count_])
                //     {
                //         //top
                //         cout << "anchor name: "<< anchor_names[count_]<<endl;
                        
                //         for(int column=0;column<8;++column)
                //         {
                //             right = max(right,tmp->box_corners_2d(0,column));
                //             left = min(left,tmp->box_corners_2d(0,column));
                //             top = min(top, tmp->box_corners_2d(1,column));
                //             bottom = max(bottom,tmp->box_corners_2d(1,column)); 
                //         }

                //         cout<<"top: "<<top<<", bottom: "<<bottom<<", left: "<<left<<", right: "<<right<<endl;
                        

                //     }
                // }
*/                
                tmp_count++;
                count_++;
            }
            #ifdef VERBOSE
            cout << "all object cuboids: "<< count_<<endl;
            #endif

            cv_bridge::CvImage cvImage_;
            cvImage_.header.stamp = ros::Time::now();
            cvImage_.header.frame_id = "detection_image";
            cvImage_.encoding = sensor_msgs::image_encodings::RGB8;
            cvImage_.image = cv::Mat(img_raw) ;
            return_pose.image_now = *cvImage_.toImageMsg();
            return_pose.count= count_;
            final_pub.publish(return_pose);
            #ifdef VERBOSE
            cout << "3D objects !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            #endif
/*
            // MatrixXd obj_bbox_coors(1, 5);                // hard coded
            // obj_bbox_coors << 188, 189, 201, 311, 0.8800; // [x y w h prob]
            // cout << xmin<<", "<<ymin <<", "<<w<<", "<<h<<", "<<prob<<endl;
            // obj_bbox_coors << xmin, ymin, w, h, prob;
            //     change matlab coordinate to c++, minus 1

            

            // Eigen::MatrixXd all_lines_raw(100, 4); // 100 is some large frame number,   the txt edge index start from 0
            
            // string base_folder = ros::package::getPath("detect_3d_cuboid") + "/data/";
            // int frame_index = 1;
            // char frame_index_c[256];
            // sprintf(frame_index_c, "%04d", frame_index); // format into 4 digit
            // read_all_number_txt(base_folder + "edge_detection/LSD/" + frame_index_c + "_edge.txt", all_lines_raw);
            

            //     done =false;
            // cout << all_object_cuboids << endl;
            // }
*/  
        }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_3d_cuboid");
    Box_handler bh;
    ros::spin();
/*
    // ros::Rate loop_rate(5);
    // while (nh.ok()) {
    //     pub.publish(msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    // ros::Rate loop_rate(10);
    // while(nh.ok())
    // {
        
        
    //     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img).toImageMsg();
        
    //     pub_img.publish(msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
    //     // cout << "publishing"<<endl;
        
        
        
    //     // // while(nh.ok()){
    //     // image_transport::ImageTransport it(nh);
    //     // image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
        
    //     ros::spinOnce();
    //     loop_rate.sleep();

    // }
    
    // }


    

    
    // cv_bridge::CvImage img_bridge;
    // sensor_msgs::Image img_msg; // >> message to be sent

    // std_msgs::Header header; // empty header
    // header.seq = 001; // user defined counter
    // header.stamp = ros::Time::now(); // time
    // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, rgb_img);
    // img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    // pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

    // read edges
    

    
    

    // ca::Profiler::print_aggregated(std::cout);
*/
    return 0;
}