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
#include <string>
#include <cstdio>
#include <zmq.hpp>
#include "opencv2/opencv.hpp"

#include<opencv2/core/core.hpp>

#include<System.h>
#include "mqtt/async_client.h"
#include "Tracking.h"

using namespace std;
using namespace std::chrono;

const std::string address { "tcp://192.168.87.250:1883" };

const string TOPIC { "robot/sensors/orbslam" };
const int	 QOS = 1;

const auto PERIOD = seconds(5);

const int MAX_BUFFERED_MSGS = 120;	// 120 * 5sec => 10min off-line buffering

const string PERSIST_DIR { "data-persist" };



int main(int argc, char **argv){
   
    if(argc != 5){
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    mqtt::async_client cli(address, "", MAX_BUFFERED_MSGS, PERSIST_DIR);

	mqtt::connect_options connOpts;
	connOpts.set_keep_alive_interval(MAX_BUFFERED_MSGS * PERIOD);
	connOpts.set_clean_session(true);
	connOpts.set_automatic_reconnect(true);

	// Create a topic object. This is a conventience since we will
	// repeatedly publish messages with the same parameters.
	mqtt::topic top(cli, TOPIC, QOS, true);

    // calibration and rectification of cameras (SLAM)

    // initialize the zmq context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REQ);

    // connect to the image server
    std::printf("Connecting to server... \n");
    socket.connect ("tcp://192.168.87.23:5555");

    // create a request object
    zmq::message_t request(5);
    memcpy(request.data(), "Hello", 5);

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
    cv::Mat Tcw; // added to save realtime values
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


    // Create SLAM system. It initializes all system threads and gets ready to process frames. (SLAM)
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true,(bool)atoi(argv[4]));
    //ORB_SLAM2::Tracking yolo (SLAM&);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
 

    // Main loop (contains code for SLAM and Path Planning)

    try {


        cout << "Connecting to server '" << address << "'..." << flush;
        cli.connect(connOpts)->wait();
        cout << "OK\n" << endl;


        cv::Mat imLeft, imRight, frame, imLeftRect, imRightRect;

        for(int timeStamps = 0; timeStamps < 6000 ; timeStamps++) {

            socket.send(request);

            // get the reply
            zmq::message_t reply;
            socket.recv(&reply);
            std::vector<uchar> buffer;

            // store the reply data into an image structure
            cv::Mat frame(240, 640, CV_8UC3, reply.data());

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
            Tcw = SLAM.TrackStereo(imLeftRect,imRightRect,timeStamps); // added to save realtime values
            SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information // added to save realtime values
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information // added to save realtime values
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc); // added to save realtime values

            float disp_x = twc.at<float>(0); // added to save realtime values
            float disp_y = twc.at<float>(1); // added to save realtime values
            float disp_z = twc.at<float>(2); // added to save realtime values
            float rot_x = q[0]; // added to save realtime values
            float rot_y = q[1]; // added to save realtime values
            float rot_z = q[2]; // added to save realtime values
            float rot_w = q[3]; // added to save realtime values

            string payload =
                to_string(disp_x) + "," + to_string(disp_y) + "," + to_string(disp_z)+ "," +
                to_string(rot_x) + "," + to_string(rot_y) + "," + to_string(rot_z) + "," + to_string(rot_w);
            top.publish(std::move(payload));

        }

        cout << "\nDisconnecting..." << flush;
        cli.disconnect()->wait();
        cout << "OK" << endl;
    }
    catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
        return 1;
    }

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}
