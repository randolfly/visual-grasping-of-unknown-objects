/*
 * jaco_kinect_calibration_main.cpp
 *
 *  Created on: Jul 7, 2015
 *      Author: raj
 */

/////////////////////////////////////////////// INCLUDES

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <cstdlib>
#include <Eigen/Geometry>
#include <math.h>
#define PIOVER180 (3.14159265359/180.0) // for fromEuler()
#include <Eigen/Dense>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
//#include "std_msgs/String.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
//moveit packages


//boost bind to use multiple arguments
#include "boost/bind.hpp"
#include "boost/shared_ptr.hpp"
#include "boost/ref.hpp"

//camera based headers
#include "calculateTransformations.h"
#include "KinectCalibrator.h"
#include "GrayImage.hh"
#include "image_converter.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <MRSysJacoMoveit.h>
#include <MRSysJacoCalibration.h>
using namespace ros;


int main(int argc, char ** argv)
{

  init(argc,argv,"MRSys_Jaco_kinect_calib");
  // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();
  MRSysJacoCalibration jaco_kinect;
  MRSysJacoMoveit      arm_declared("jaco_arm");

	double x0,x1,y0,y1,height;
	int    camera;
	 std::cout << "\nInitial check to proceed with calibration\n\
	     Press (y) To do New calibration *(NOTE:) remove the old -coordsNquad- file before pressing y\n\
	     press (n) to recalculate the transformation from the existing -coordsNquad- file--\n" << std::endl;
		std::string inputs;
		std::cin >> inputs;
		if (!std::strcmp(inputs.c_str(),"y"))
		{


   FILE *f_coordsNquad;
   int count = openFiles(f_coordsNquad);

   ros::Duration(1.0).sleep();



    std::cout << "\n\n\n\n\n\n";
    std::cout << " ----        Ok, start the calibration!        ----\n" << std::endl;

    std::cout << "\nGive a crude estimation of the direction of camera -- that is, a\n\
     number in the range 0-7, where 0 means positive y-axis, 2 is\n\
     positive x-axis, 4 is negative y-axis and so on. If you don't\n\
     want to specify a direction enter 8 and the hand will point down-\n\
     wards." << std::endl;
       std::cin >> camera;


            // Define the pose space for generation of poses and number of positions
            // x0 = -30; x1 = -50; y0 = -20; y1 = -40; height = 50;
             int positions = 30;
		  x0 = -20; x1 = -40; y0 = -50; y1 = -80; height = 110;


             for (; count < positions;)

                {
                       // Generate a random position
                 //printf("for loop counting start %d/%d\n", count, positions);

            	 jaco_kinect.random_pose =   jaco_kinect.jaco_random_poses(x0,x1,y0,y1,height,camera);

            	 jaco_kinect.desired_pose.position.x = jaco_kinect.random_pose(0);
            		jaco_kinect.desired_pose.position.y =jaco_kinect.random_pose(1);
            			jaco_kinect.desired_pose.position.z =jaco_kinect.random_pose(2);
            				jaco_kinect.desired_pose.orientation.x =jaco_kinect.random_pose(3);
            					jaco_kinect.desired_pose.orientation.y =jaco_kinect.random_pose(4);
            						jaco_kinect.desired_pose.orientation.z =jaco_kinect.random_pose(5);
            							jaco_kinect.desired_pose.orientation.w =jaco_kinect.random_pose(6);

ROS_INFO("pose published to JACO ARM (%.5lf,%.5lf,%.5lf) orientation(%.5lf,%.5lf,%.5lf,%.5lf)",jaco_kinect.desired_pose.position.x, jaco_kinect.desired_pose.position.y, jaco_kinect.desired_pose.position.z,jaco_kinect.desired_pose.orientation.x, jaco_kinect.desired_pose.orientation.y, jaco_kinect.desired_pose.orientation.z, jaco_kinect.desired_pose.orientation.w);


ros::Duration(0.5).sleep();
	bool success= arm_declared.jaco_desired_pose(jaco_kinect.desired_pose);


    		     //printf("\n Trying to move the JacoArm to Position %d/%d\n", count, positions);
    		     //ROS_INFO("Arm execution successful for calibration process press 0 ");
    		   //  bool success= arm_declared.jaco_moveit_plan(jaco_kinect.desired_pose);
    		 	std::cout <<success<<std::endl;



if (success)
    		     {
	 // Search for the IR-light and save all necessary data
			ros::Duration(10.0).sleep();
    		       std::cout << "   ... Robot arm moved, searching for the IR-light" <<std::endl;
    		       std::cout << "   ... ";
    		       writeArmNLedCoords(jaco_kinect.random_pose, f_coordsNquad, count);
                   printf("Obtained IR LIGHT Coordinates for %d/%d\n", count+1, positions);

                   printf("Obtained IR LIGHT Coordinates corresponding to the Jaco Arm pose");
    		     }
else
    		     {
    		       std::cout << "   ... not able to move the arm Failed to register the ir coord, " <<
    		 	   std::endl;
    		       std::cout << "  going for the next pose plan and execution ";
    		       ros::Duration(0.3).sleep();
    		       continue;
    		     }




    		     ros::Duration(0.5).sleep();

  	    // count=count+1;
         printf("\n Next ARM Pose To be Moved %d/%d\n", count, positions);

                }


             // Close the files
             fclose(f_coordsNquad);

             std::cout << "Calibration Completed "<<std::endl;
		}

             // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             // %%%%%%%%%%%%%%%%%%% CALCULATE THE TRANSFORMATIONS %%%%%%%%%%%%%%%%%%%%
             // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             Eigen::Matrix4d trans_rot =calculateTransformation("coordsNquad.txt");
             jaco_kinect.CameratoRobot= jaco_kinect.StaticTransform(trans_rot);
             arm_declared.~MRSysJacoMoveit();
             return 0;
}


