// /**
// * This file is part of ORB-SLAM2.
// *
// * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// * For more information see <https://github.com/raulmur/ORB_SLAM2>
// *
// * ORB-SLAM2 is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * ORB-SLAM2 is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
// */

// #include <unistd.h>

// #include<iostream>
// #include<algorithm>
// #include<fstream>
// #include<chrono>

// #include<opencv2/core/core.hpp>

// #include<System.h>

// using namespace std;

// void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
//                 vector<double> &vTimestamps);

// int main(int argc, char **argv)
// {
//     if(argc != 4)
//     {
//         cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
//         return 1;
//     }

//     // Retrieve paths to images
//     vector<string> vstrImageFilenames;
//     vector<double> vTimestamps;
//     string strFile = string(argv[3])+"/rgb.txt";
//     LoadImages(strFile, vstrImageFilenames, vTimestamps);

//     int nImages = vstrImageFilenames.size();

//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
//     SLAM.latency = 0.0f;
//     SLAM.experiment = false;
//     // Vector for tracking time statistics
//     vector<float> vTimesTrack;
//     vTimesTrack.resize(nImages);

//     cout << endl << "-------" << endl;
//     cout << "Start processing sequence ..." << endl;
//     cout << "Images in the sequence: " << nImages << endl << endl;

//     // Main loop
//     cv::Mat im;
//     for(int ni=0; ni<nImages; ni++)
//     {
//         // Read image from file
//         im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
//         double tframe = vTimestamps[ni];

//         if(im.empty())
//         {
//             cerr << endl << "Failed to load image at: "
//                  << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
//             return 1;
//         }

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif

//         // Pass the image to the SLAM system
//         SLAM.TrackMonocular(im,tframe);

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif

//         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//         vTimesTrack[ni]=ttrack;

//         // Wait to load the next frame
//         double T=0;
//         if(ni<nImages-1)
//             T = vTimestamps[ni+1]-tframe;
//         else if(ni>0)
//             T = tframe-vTimestamps[ni-1];

//         if(ttrack<T)
//             usleep((T-ttrack)*1e6);
//     }

//     cout << "Stopping all threads..." << endl;
//     // Stop all threads
//     SLAM.Shutdown();
//     cout << "done" << endl;

//     // Tracking time statistics
//     sort(vTimesTrack.begin(),vTimesTrack.end());
//     float totaltime = 0;
//     for(int ni=0; ni<nImages; ni++)
//     {
//         totaltime+=vTimesTrack[ni];
//     }
//     cout << "-------" << endl << endl;
//     cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
//     cout << "mean tracking time: " << totaltime/nImages << endl;

//     // Save camera trajectory
//     cout << SLAM.offload_num <<endl;
//     SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

//     return 0;
// }

// void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
// {
//     ifstream f;
//     f.open(strFile.c_str());

//     // skip first three lines
//     string s0;
//     getline(f,s0);
//     getline(f,s0);
//     getline(f,s0);

//     while(!f.eof())
//     {
//         string s;
//         getline(f,s);
//         if(!s.empty())
//         {
//             stringstream ss;
//             ss << s;
//             double t;
//             string sRGB;
//             ss >> t;
//             vTimestamps.push_back(t);
//             ss >> sRGB;
//             vstrImageFilenames.push_back(sRGB);
//         }
//     }
// }

/*
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <unistd.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


void LoadTxt(string &s0, vector<string> &all)
{
    
    string::size_type begin,end;
    string pattern = ",";
    begin =0;
    end = s0.find(pattern);

    while(end != std::string::npos){
        if(end-begin!=0){
            all.push_back(s0.substr(begin,end-begin));
        }
        begin = end+pattern.size();
        end = s0.find(pattern,begin);
    }

    return;

}

int main(int argc, char **argv)
{
    // if(argc != 3)
    // {
    //     cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
    //     return 1;
    // }
    
    string dataset = "fr1_xyz";
    string strFile = "/media/brian/7385-C745/"+dataset+"/select_image.txt";
    ifstream f;
    f.open(strFile.c_str());
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    SLAM.experiment = false;
    string s0;
    // f >> s0;
    std::vector<string> all_number;
    // cout << s0 << endl;
    std::vector<float> accuracy;
    float all_acc = 0;
    while(f>>s0){
        all_number.clear();
        LoadTxt(s0, all_number);
        cout <<"host: "<< "/media/brian/7385-C745/"+dataset+"/image_raw/"+all_number[0]+".png" << endl;
        cv::Mat im_host = cv::imread("/media/brian/7385-C745/"+dataset+"/image_raw/"+all_number[0]+".png",CV_LOAD_IMAGE_UNCHANGED);

        cv::Mat im;
        for(int ni = 1; ni<all_number.size();++ni){
            SLAM.TrackMonocular(im_host,0);
            usleep(0.5);
            im = cv::imread("/media/brian/7385-C745/"+dataset+"/image_raw/"+all_number[ni]+".png",CV_LOAD_IMAGE_UNCHANGED);
            SLAM.TrackMonocular(im,1);
            usleep(0.5);
            // SLAM.Reset();
        }

        float acc_tmp = SLAM.initial_num/(all_number.size()-1);
        cout << "accuracy: "<< acc_tmp <<endl;

        accuracy.push_back(acc_tmp);
        all_acc += acc_tmp;
    }

    cout<< "final accuracy"<< all_acc/accuracy.size()<<endl;

    // cout << s0 << endl;
    // for(vector<int>::iterator iter = all_number.begin();iter!=all_number.end();++iter){
    //     cout << *iter << " ";

    // }
    
    // f >> s0;
    // cout << s0 << endl;

    

    
    // skip first three lines
    

//     getline(f,s0);
//     getline(f,s0);
//     getline(f,s0);

//     while(!f.eof())
//     {
//         string s;
//         getline(f,s);
//         if(!s.empty())
//         {
//             stringstream ss;
//             ss << s;
//             double t;
//             string sRGB;
//             ss >> t;
//             vTimestamps.push_back(t);
//             ss >> sRGB;
//             vstrImageFilenames.push_back(sRGB);
//         }
//     }

//     // Retrieve paths to images
//     vector<string> vstrImageFilenames;
//     vector<double> vTimestamps;
//     string strFile = string(argv[3])+"/rgb.txt";

//     LoadImages(strFile, vstrImageFilenames, vTimestamps);

//     int nImages = vstrImageFilenames.size();

//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
//     SLAM.latency = 0.0f;
//     SLAM.experiment = false;
//     // Vector for tracking time statistics
//     vector<float> vTimesTrack;
//     vTimesTrack.resize(nImages);

//     cout << endl << "-------" << endl;
//     cout << "Start processing sequence ..." << endl;
//     cout << "Images in the sequence: " << nImages << endl << endl;

//     // Main loop
//     

//     cout << "Stopping all threads..." << endl;
//     // Stop all threads
//     SLAM.Shutdown();
//     cout << "done" << endl;

//     // Tracking time statistics
//     sort(vTimesTrack.begin(),vTimesTrack.end());
//     float totaltime = 0;
//     for(int ni=0; ni<nImages; ni++)
//     {
//         totaltime+=vTimesTrack[ni];
//     }
//     cout << "-------" << endl << endl;
//     cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
//     cout << "mean tracking time: " << totaltime/nImages << endl;

//     // Save camera trajectory
//     cout << SLAM.offload_num <<endl;
//     // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
