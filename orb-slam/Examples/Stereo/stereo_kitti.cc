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
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "Tracking.h"
#include "MQTTClient.h"
#include "linux.cpp"

using namespace std;


int main(int argc, char **argv){
   
    if(argc != 5){
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    IPStack ipstack = IPStack();
    const char* topic = "mbed-sample";

    MQTT::Client<IPStack, Countdown> client = MQTT::Client<IPStack, Countdown>(ipstack);

    const char* hostname = "192.168.87.250";
    int port = 1883;
    printf("Connecting to %s:%d\n", hostname, port);
    int rc = ipstack.connect(hostname, port);
    if (rc != 0){
        printf("rc from TCP connect is %d\n", rc);
    }

    printf("MQTT connecting\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char*)"mbed-icraggs";
    rc = client.connect(data);
    if (rc != 0){
        printf("rc from MQTT connect is %d\n", rc);
    }
    printf("MQTT connected\n");


    // calibration and rectification of cameras (SLAM)

    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened()){
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }



    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);



    // Intializing camera feed (SLAM)
    cv::VideoCapture camera0(2);
    if (!camera0.isOpened()) {
        cerr << endl  <<"Could not open camera feed."  << endl;
        return -1;
    }




    // Create SLAM system. It initializes all system threads and gets ready to process frames. (SLAM)
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true,(bool)atoi(argv[4]));
    //ORB_SLAM2::Tracking yolo (SLAM&);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
 

    // Main loop (contains code for SLAM and Path Planning)



    cv::Mat imLeft, imRight, frame, imLeftRect, imRightRect;

    for(int timeStamps = 0; timeStamps < 600 ; timeStamps++) {
        camera0 >> frame;
        imLeft = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));

        imRight = frame(cv::Rect(frame.cols/2, 0, frame.cols / 2, frame.rows));

        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);

        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);


        cv::Rect rect1(0, 0, 9, 240);
        cv::Rect rect2(0, 0, 320, 61);
        cv::Rect rect3(211, 0, 109, 240);
        cv::Rect rect4(0, 185, 320, 55);

        cv::Rect rect11(0, 0, 6, 240);
        cv::Rect rect22(0, 0, 320, 67);
        cv::Rect rect33(214, 0, 106, 240);
        cv::Rect rect44(0, 190, 320, 50);

        cv::rectangle(imLeftRect, rect1, cv::Scalar(0, 0, 0),CV_FILLED);
        cv::rectangle(imLeftRect, rect2, cv::Scalar(0, 0, 0),CV_FILLED);
        cv::rectangle(imLeftRect, rect3, cv::Scalar(0, 0, 0),CV_FILLED);
        cv::rectangle(imLeftRect, rect4, cv::Scalar(0, 0, 0),CV_FILLED);

        cv::rectangle(imRightRect, rect11, cv::Scalar(0, 0, 0),CV_FILLED);
        cv::rectangle(imRightRect, rect22, cv::Scalar(0, 0, 0),CV_FILLED);
        cv::rectangle(imRightRect, rect33, cv::Scalar(0, 0, 0),CV_FILLED);
        cv::rectangle(imRightRect, rect44, cv::Scalar(0, 0, 0),CV_FILLED);

        // Pass the images to the SLAM system (SLAM)
        SLAM.TrackStereo(imLeftRect,imRightRect,timeStamps);
        SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

        SLAM.SaveTrajectoryTUM("CameraTrajectory2.txt");

        // Real-time values of position and orientation from SLAM (these values would be used in Path Planning in a closed feedback loop to see if our robot is moving in the right direction and orientation).
        float disp_x = SLAM.x;   // detects if robot is moving in left or right in a straight direction
        float disp_y = SLAM.y;	 // detects if robot is moving up or down in a striaght direction (not used in our case)
        float disp_z = SLAM.z;   // detects if robot is moving straight or backawards in a straight direction
        float rot_x = SLAM.rx;	// roll orientation (not used yet but can be used to detect uphill or downhill)
        float rot_y = SLAM.ry;	// yaw orientation (tells us the angle at which robot turns left or right)
        float rot_z = SLAM.rz;	// pitch orientation (not used yet but can be used to detect uphill or downhill)


    }


    // Stop all threads
    SLAM.Shutdown();
    return 0;
}