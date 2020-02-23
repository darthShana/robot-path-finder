
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <Tracking.h>

#include <opencv2/core/core.hpp>

#include <System.h>


#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <nlohmann/json.hpp>


using namespace curlpp::options;



class RobotControl {
  public:
    // Assembles the client's payload, sends it and presents the response back
    // from the server.
    std::string goStraight() {
      std::cout << "in straight" << std::endl;
      try
      {
          // That's all that is needed to do cleanup of used resources (RAII style).
          curlpp::Cleanup myCleanup;

          // Our request to be sent.
          curlpp::Easy request;

          // Set the URL.
          request.setOpt<Url>("http://localhost:5000/robot/commands");
          std::list<std::string> header;
          header.push_back("Content-Type: application/json");

          nlohmann::json j =
              {
                  {"thrust", 1350},
                  {"heading", 1500},
              };

          std::string body = j.dump();

          request.setOpt(new curlpp::options::HttpHeader(header));
          request.setOpt(new curlpp::options::PostFields(body));
          request.setOpt(new curlpp::options::PostFieldSize(body.length()));

          // Send request and get a result.
          // By default the result goes to standard output.
          request.perform();
      }
      catch(curlpp::RuntimeError & e)
      {
          std::cout << e.what() << std::endl;
      }

      catch(curlpp::LogicError & e)
      {
          std::cout << e.what() << std::endl;
      }
      return "";
    }

    std::string stop() {
      std::cout << "in stop" << std::endl;
      try
            {
                // That's all that is needed to do cleanup of used resources (RAII style).
                curlpp::Cleanup myCleanup;

                // Our request to be sent.
                curlpp::Easy request;

                // Set the URL.
                request.setOpt<Url>("http://localhost:5000/robot/commands");
                std::list<std::string> header;
                header.push_back("Content-Type: application/json");

                nlohmann::json j =
                    {
                        {"thrust", 1500},
                        {"heading", 1500},
                    };

                std::string body = j.dump();

                request.setOpt(new curlpp::options::HttpHeader(header));
                request.setOpt(new curlpp::options::PostFields(body));
                request.setOpt(new curlpp::options::PostFieldSize(body.length()));

                // Send request and get a result.
                // By default the result goes to standard output.
                request.perform();
            }
            catch(curlpp::RuntimeError & e)
            {
                std::cout << e.what() << std::endl;
            }

            catch(curlpp::LogicError & e)
            {
                std::cout << e.what() << std::endl;
            }
      return "";
    }

    std::string turnAngle(float Angle) {
      std::cout << "in turn" << std::endl;
      try
            {
                // That's all that is needed to do cleanup of used resources (RAII style).
                curlpp::Cleanup myCleanup;

                // Our request to be sent.
                curlpp::Easy request;

                // Set the URL.
                request.setOpt<Url>("http://localhost:5000/robot/commands");
                std::list<std::string> header;
                header.push_back("Content-Type: application/json");

                nlohmann::json j =
                    {
                        {"thrust", 1330},
                        {"heading", Angle},
                    };

                std::string body = j.dump();

                request.setOpt(new curlpp::options::HttpHeader(header));
                request.setOpt(new curlpp::options::PostFields(body));
                request.setOpt(new curlpp::options::PostFieldSize(body.length()));

                // Send request and get a result.
                // By default the result goes to standard output.
                request.perform();
            }
            catch(curlpp::RuntimeError & e)
            {
                std::cout << e.what() << std::endl;
            }

            catch(curlpp::LogicError & e)
            {
                std::cout << e.what() << std::endl;
            }
      return "";
    }

  std::string reverseAngle(float Angle) {
      std::cout << "in reverse" << std::endl;
      try
            {
                // That's all that is needed to do cleanup of used resources (RAII style).
                curlpp::Cleanup myCleanup;

                // Our request to be sent.
                curlpp::Easy request;

                // Set the URL.
                request.setOpt<Url>("http://localhost:5000/robot/commands");
                std::list<std::string> header;
                header.push_back("Content-Type: application/json");

                nlohmann::json j =
                    {
                        {"thrust", 1650},
                        {"heading", Angle},
                    };

                std::string body = j.dump();

                request.setOpt(new curlpp::options::HttpHeader(header));
                request.setOpt(new curlpp::options::PostFields(body));
                request.setOpt(new curlpp::options::PostFieldSize(body.length()));

                // Send request and get a result.
                // By default the result goes to standard output.
                request.perform();
            }
            catch(curlpp::RuntimeError & e)
            {
                std::cout << e.what() << std::endl;
            }

            catch(curlpp::LogicError & e)
            {
                std::cout << e.what() << std::endl;
            }
      return "";
    }


};


int main(int argc, char **argv) {

  RobotControl controller;
		
  //string test1= controller.goStraight();
  //sleep(1);
  //string test2 = controller.turnAngle(2000);
  //sleep(1);
  //string test3 = controller.turnAngle(1000);
 // sleep(1);
  string test4;
  int x = 1700;
  
   //char ch;
  //  system("stty raw");//seting the terminal in raw mode

char ch;
    do{
        ch=getchar();
         if(ch==65){
            printf("You pressed UP key\n");
            test4 = controller.goStraight();}
         else if(ch==66){
            printf("You pressed DOWN key\n");
            test4 = controller.stop();}
         else if(ch==67){
            x = x+50;
            test4 = controller.turnAngle(x);
            printf("You pressed RIGHT key\n");}
         else if(ch==68){
           x = x-50;
            test4 = controller.turnAngle(x);
            printf("You pressed LEFT key\n");}
 
 
 
    }while(ch!='e');


  // to read the previously saved SLAM Trajectory (for path planning)
  std::ifstream file("/home/pi/orb2/CameraTrajectory.txt");

  cv::Mat Traj;
  int rows = 0;
  std::string line;
  while (std::getline(file, line)) {

    std::istringstream stream(line);
    while (stream) {
        string s;
        if (!std::getline(stream, s, ' ')) break;
        Traj.push_back(std::stof(s));
    }

    rows++;
  }


  Traj = Traj.reshape(1, rows);

  char State;
  char StateOrder[1000];

  float start_turning_angle[1000];
  float start_turning_x[1000];
  float start_turning_z[1000];
  float stop_turning_angle[1000];
  float stop_turning_x[1000];
  float stop_turning_z[1000];
  float start_straight_x[1000];
  float start_straight_z[1000];
  float start_straight_angle[1000];
  float stop_straight_angle[1000];
  float stop_straight_x[1000];
   float stop_straight_z[1000];
  float disp_x_prev=0;
  float disp_z_prev=0;
  float buffer_x=0;
  float buffer_z=0;
  float buffer_angle=0;
  int buffer =0;

  float straight;
  float turnDistance;
  float turnFactor;

  int k=0;
  int kk=0;
  int m=0;
  int current_m=0;
  int zz=0;
  int z=0;
  int w =0;
  int ww=0;
  int mutex=0;
  int mutex1=1;
  int mutex2=0;
  int mutex3=0;
  int mutex4=0;
  int s=0;
  int ss=0;

  


// going throught the saved trajectory to break it down into waypoints.
// we detect starting and ending points of all the straight paths and turns 


// loop through the entire trajectory file
  for (int i=1; i < Traj.rows; i++){
// we do not save waypoints if distance covered is very small (or the robot is stationary).
    if((sqrt(pow(Traj.at<float>(m,3) - Traj.at<float>(i,3) ,2) + pow(Traj.at<float>(m,1)  - Traj.at<float>(i,1) ,2))) > 0.035){
// detect and save straight paths (very small change in angle)  
    if (abs((abs(Traj.at<float>(i,5))-abs(Traj.at<float>(i-1,5)))) <= 0.004){
    
    start_straight_angle[w] = Traj.at<float>(m,5);
    start_straight_x[w] = Traj.at<float>(m,1);
    start_straight_z[w] = Traj.at<float>(m,3);
    stop_straight_angle[w] = Traj.at<float>(i,5);
    stop_straight_x[w] = Traj.at<float>(i,1);
    stop_straight_z[w] = Traj.at<float>(i,3);
   // printf ("%f    yoo  %f", stop_straight_angle[w],start_straight_angle[w]);
      if (mutex3 == 1){
        s = s+1;
        mutex3=0;
      }
    mutex4=0;
    mutex =1;
   
   StateOrder[s] = 'f';
   
    }
// save starting of turns 
	  if (abs((abs(Traj.at<float>(i,5))-abs(Traj.at<float>(i-1,5)))) > 0.004){
		if(mutex ==1){
    w = w+1;
    printf("%c",StateOrder[s]);
    s = s+1;
    
  }
    
    mutex =0;
		start_turning_angle[k] = Traj.at<float>(i-1,5);
    start_turning_x[k] = Traj.at<float>(i-1,1);
    start_turning_z[k] = Traj.at<float>(i-1,3);
   
    printf("%c",StateOrder[s]);
printf("  %f\n",start_turning_angle[k]);
		

		//if (i < Traj.rows-2){
		  //k=k+1;
		//}
	    //printf("%f",Traj.at<float>(i,5));
      
// once the starting of turn is detected, a for loop is used to scan the values after that
// to detect ending of the turn.
		for (int j= i+1; j < Traj.rows; j++){

			//if((sqrt(pow(Traj.at<float>(i,3) - Traj.at<float>(j,3) ,2) + pow(Traj.at<float>(i,1)  - Traj.at<float>(j,1) ,2))) > 0.03){
				if (j == Traj.rows-1){
					i = j+1;
					stop_turning_angle[z] = Traj.at<float>(j,5);
          stop_turning_x[z] = Traj.at<float>(j,1);
          stop_turning_z[z] = Traj.at<float>(j,3);
          printf("  yyS: %f\n",stop_turning_angle[z]);
          if ( (sqrt(pow(start_turning_x[k] - stop_turning_x[z] ,2) + pow(start_turning_z[k]  - stop_turning_z[z] ,2))) <= 0.1){
          mutex4 =1;
          w = w-1;
          s = s-1;
           }
          
          
                }

				if (abs((abs(Traj.at<float>(j,5))-abs(Traj.at<float>(j-1,5)))) <= 0.004 && j != (Traj.rows-1)){
					stop_turning_angle[z] = Traj.at<float>(j-1,5);
          stop_turning_x[z] = Traj.at<float>(j-1,1);
          stop_turning_z[z] = Traj.at<float>(j-1,3);
          printf("  S: %f\n",stop_turning_angle[z]);
					if (j != Traj.rows-1){
     
// sometimes in a straight path, small angles may be detected. The code below makes sure to
// make those movements a part of a straight path and not consider them turns.
           if ( (sqrt(pow(start_turning_x[k] - stop_turning_x[z] ,2) + pow(start_turning_z[k]  - stop_turning_z[z] ,2))) <= 0.1){
          
            
             if (s == 0){
            
             i = j;
             m = 0;
             }
             if (s != 0){
             
             w = w-1;
             s = s-1;
             i =j;
             }
            }
            if ( (sqrt(pow(start_turning_x[k] - stop_turning_x[z] ,2) + pow(start_turning_z[k]  - stop_turning_z[z] ,2))) > 0.1){
            
            StateOrder[s] = 't';
						z=z+1;
						m = j-1;
						i = j;
            mutex3 =1;
            k=k+1;
           
            }
          }
          
    
					break;
                }
            
         }
       
      }
    }
  }
  
  
 
  if (mutex4 == 1){
    
    stop_straight_x[w] = stop_turning_x[z];
    stop_straight_z[w] = stop_turning_z[z];
    stop_straight_angle[w] = stop_turning_angle[z];
    
  }

//printf(" %f,%f,%f,%f",stop_straight_angle[0], start_straight_angle[0], stop_turning_angle[0], start_turning_angle[0]);
printf("------------%c,%c",StateOrder[0],StateOrder[1]);

  // Housekeeping and initialization for SLAM (SLAM).
  if(argc != 5){
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
  }

  // calibration and rectification of cameras (SLAM)
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if(!fsSettings.isOpened()) {
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
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0) {
    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
    return -1;
  }

  cv::Mat M1l,M2l,M1r,M2r;
  cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
  cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

  // Intializing camera feed (SLAM)
  cv::VideoCapture camera0(0);
  if (!camera0.isOpened()) {
    cerr << endl  <<"Could not open camera feed."  << endl;
    return -1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames. (SLAM)
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true,(bool)atoi(argv[4]));


  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  

  // Main loop (contains code for SLAM and Path Planning)
  
  //double timeStamps=0;

// take input from camera and prepare it for SLAM
  cv::Mat imLeft, imRight, frame, imLeftRect, imRightRect;
  for(int timeStamps=0; timeStamps <200; timeStamps++) {
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
	//timeStamps = timeStamps + 0.033;

      if (timeStamps > 2){
        // Save new camera trajectory (SLAM). Note: this command will save the SLAM trajectory when the user will move around the lawn mower manually. This file would then be read by the Path Planning
        //code in this section to interpret it and control the motors.
      
         SLAM.SaveTrajectoryTUM("CameraTrajectory2.txt");
         
        // Real-time values of position and orientation from SLAM (these values would be used in Path Planning in a closed feedback loop to see if our robot is moving in the right direction and orientation).
        float disp_x = SLAM.x;   // detects if robot is moving in left or right in a straight direction
        float disp_y = SLAM.y;	 // detects if robot is moving up or down in a striaght direction (not used in our case)
        float disp_z = SLAM.z;   // detects if robot is moving straight or backawards in a straight direction
        float rot_x = SLAM.rx;	// roll orientation (not used yet but can be used to detect uphill or downhill)
        float rot_y = SLAM.ry;	// yaw orientation (tells us the angle at which robot turns left or right)
        float rot_z = SLAM.rz;	// pitch orientation (not used yet but can be used to detect uphill or downhill)
        int insideMap1 = SLAM.mpTracker->insideMap;
        
        //printf("%f       ",disp_x);
//printf ("%f",rot_y);

    /*    
     if(timeStamps1 == 3){
            test4 = controller.goStraight();}
      if (timeStamps1 == 6){
            test4 = controller.turnAngle(x);
            }
            if (timeStamps1 == 11){
             test4 = controller.goStraight();
            }
            if (timeStamps1 == 13){
              test4 = controller.stop();
            }
          */
    
    //printf("%d /n",mutex1);
   // printf("%f .................. %f",stop_straight_angle[ww], rot_y);
   
// use waypoints to plan the path
     if (ss<=s){
       
// for straight paths
       if(StateOrder[ss] == 'f'){
         
      
         if(mutex1=1){
          disp_x_prev=disp_x;
          disp_z_prev=disp_z;
// save the distances that need to be covered          
          straight = sqrt(pow(stop_straight_x[ww] - disp_x_prev ,2) + pow(stop_straight_z[ww]  - disp_z_prev ,2));
          mutex1 = 0;
       
         }
// check if we are inside map         
         if (insideMap1 == 1 ){
// check in real time if the distance has been covered
           if((sqrt(pow(disp_x_prev - disp_x ,2) + pow(disp_z_prev  - disp_z ,2))) < (straight - 0.25)){
              //controller.goStraight();
// if we are still in the map but current angle is different for the require angle make small 
// turn to adjust the position              
              if (rot_y > stop_straight_angle[ww] + 0.04){
                printf("left");
                controller.turnAngle(1700);
              }
              if (rot_y < stop_straight_angle[ww] - 0.04){
                controller.turnAngle(1300);
                printf("right");
              }
// if the robot is moving with the right angle. keep going straight              
              if (rot_y <= stop_straight_angle[ww] + 0.04 && rot_y >= stop_straight_angle[ww] -0.04){
                controller.goStraight();
              }
             // float remaining = sqrt(pow(disp_x_prev - disp_x ,2) + pow(disp_z_prev  - disp_z ,2));
             //printf("%f .............", remaining);
         }
// stop when distance covered and move to the next waypoint
           if((sqrt(pow(disp_x_prev - disp_x ,2) + pow(disp_z_prev  - disp_z ,2))) >= (straight-0.25)){
             controller.stop(); //completed
             ss = ss+1;
             mutex1 =1;
             disp_x_prev = disp_x;
             disp_z_prev = disp_z;
             ww = ww+1;
          
          }
         }
 // if outside map, try coming back in the map         
         if (insideMap1 == 2 || insideMap1 == 0){
           printf("out");
          if((sqrt(pow(disp_x_prev - disp_x ,2) + pow(disp_z_prev  - disp_z ,2))) < straight - 0.25){
             if (rot_y > stop_straight_angle[ww]){
             controller.turnAngle(2000);
             }
           
             if (rot_y < stop_straight_angle[ww]){
             controller.turnAngle(1000);
             }
           
            
             if (rot_y == stop_straight_angle[ww]){
            controller.goStraight();
             }
             
           } 
 // it outside map and the required distance is covered, stop the robot and move to the next waypoint          
          if((sqrt(pow(disp_x_prev - disp_x ,2) + pow(disp_z_prev  - disp_z ,2))) >= (straight-0.25)){
           
              
              controller.stop(); //completed
              ss = ss+1;
              ww = ww+1;
             mutex1 =1;
             disp_x_prev = disp_x;
             disp_z_prev = disp_z;
            
         }
          
         
         
       }
       
       
       
     }
// for turns
    if (StateOrder[ss] == 't'){
// calculate the distance that need to be covered in the turn
      turnDistance = sqrt(pow(stop_turning_x[zz] - disp_x ,2) + pow(stop_turning_z[zz]  - disp_z ,2));
// calculate the turn factor for adaptive turning
      turnFactor = turnDistance / abs(stop_turning_angle[zz] -rot_y);
      printf("%f",rot_y);
      printf("    %f", turnDistance);
      //printf("             %f", disp_z);
      printf ("                      %f", turnFactor);

// see if we have reached the required angle in realtime and stop
       if (abs(stop_turning_angle[zz] +0.5) < abs(rot_y) ){
        controller.stop();
        kk = kk+1;
        zz = zz+1;
        ss =ss+1;
        mutex1=1;
        mutex2 = 1;
      }
// change angle of the turn based on the turnfactor. (only implemented for left turns)
      if (mutex2 == 0){
      
    
      if (turnFactor <= 3){
        
        controller.turnAngle(2000);
      }
       if (turnFactor <= 4 && turnFactor >3){
        
        controller.turnAngle(1900);
      }
      if (turnFactor <= 6 && turnFactor >4){
        
        controller.turnAngle(1800);
      }
      if (turnFactor > 6){
        
        controller.turnAngle(1700);
      }
      
     }
     
        
    }

  }
  
  
}
}

   // Stop all threads
    
  SLAM.Shutdown();
   return 0;
}


