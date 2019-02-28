/////////////////////////////////////////////// INCLUDES
float above_table = 0.25;//25cm

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <cstdlib>
#include <sstream>
#include "std_msgs/String.h"
#include <Eigen/Geometry>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_tools/solid_primitive_dims.h>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/callback_queue.h>


//moveit packages
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/controller_manager/controller_manager.h>
//boost bind to use multiple arguments
#include "boost/bind.hpp"
#include "boost/shared_ptr.hpp"
#include "boost/ref.hpp"

//gripper
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <trajectory_msgs/JointTrajectory.h>


//Action client and control msgs
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace ros;
typedef moveit::planning_interface::MoveGroup MOVEPI;
typedef moveit::planning_interface::MoveGroup::Plan  MOVEPLAN;

static volatile bool keypressed = false;
bool grasp_on_progress = false;
static boost::shared_ptr< boost::thread > keypressThread;


// Create simple action client for gripper handling
actionlib::SimpleActionClient<control_msgs::GripperCommandAction> *gripper_ac;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *jaco_trajec_ac;

static void
keypressLoop()
{
  while (!ros::isShuttingDown())
  {
    // Check for input 
    std::cin.get();
    keypressed = true;
  }
}

static void
keypressWait()
{
  ros::Rate r(100); // 100 hz
  
  ros::spinOnce();
  keypressed = false;
  std::cout << "Waiting for input to continue!" << std::endl;
  while (!keypressed)
  {
    // Sleep 0.01s
    r.sleep();

    ros::spinOnce();
  }
}

bool jaco_move_gripper_to(double position) {
	while (!gripper_ac->waitForServer(ros::Duration(2.0))){
		ros::spinOnce();
	}
	control_msgs::GripperCommandGoal close_goal;
	close_goal.command.position = position;
	close_goal.command.max_effort = -1.0; // No limit for effort (negative value)
	gripper_ac->sendGoal(close_goal);
	gripper_ac->waitForResult();
	if (gripper_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		return true;
	}
	return false;
}
bool jaco_open_gripper() {
	if (!jaco_move_gripper_to(0.0)) {
		ROS_WARN("Gripper opening failed!");
		return false;
	}
	ROS_INFO("Gripper opening succeeded!");
	return true;
}

bool jaco_close_gripper() {
	if (!jaco_move_gripper_to(1.0)) {
		ROS_WARN("Gripper closing failed!");
		return false;
	}
	ROS_INFO("Gripper closing succeeded!");
	return true;
}
geometry_msgs::Pose jaco_random_poses(double x0, double x1,double y0, double y1, double height)
{
	 double const Z_MIN = 10; // Will not generate positions lower than Z_MIN
	 srand((unsigned)time(0)); // Initialize the random generator
	 double range_x = x1 - x0 + 1;
         double range_y = y1 - y0 + 1;

	  double range_z = 1;
	  if (height > Z_MIN)
	  {
	    range_z=(height-Z_MIN)+1;
	  }

	  double x,y,z;

	  x = x0 + (range_x*rand()/(RAND_MAX + 1.0));
	  y = y0 + (range_y*rand()/(RAND_MAX + 1.0));
	  z = Z_MIN + (range_z*rand()/(RAND_MAX + 1.0));
          geometry_msgs::Pose random_pose;
          random_pose.position.x= x/100;
          random_pose.position.y= y/100;
          random_pose.position.z= z/100;
          random_pose.orientation.x=0.0;
          random_pose.orientation.y=0.0;
          random_pose.orientation.z=0.0;
          random_pose.orientation.w=1.0;

          return random_pose;

}

bool jaco_driver_trajec_execute(trajectory_msgs::JointTrajectory jaco_trajectory)
{ 
        control_msgs::FollowJointTrajectoryGoal movement_goal;
	movement_goal.trajectory = jaco_trajectory;
	jaco_trajec_ac->sendGoal(movement_goal);
        //ROS_INFO("Sending follow joint trajectory execution command");
	while (!jaco_trajec_ac->getState().isDone() && ros::ok())
	 {
		    ros::Duration(0.1).sleep();
		    ROS_INFO("Execution Active");
	 }
	if (jaco_trajec_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Trajectory execution succeeded!");
		return true;
	}
	else
	{
		ROS_WARN("Trajectory execution failed!");
		return false;
	}
}
bool jaco_move_to_position( geometry_msgs::Pose jaco_pose, boost::shared_ptr<MOVEPI> group_jaco) {
	ROS_INFO("Moving to position (%.5lf, %.5lf, %.5lf)!",jaco_pose.position.x, jaco_pose.position.y, jaco_pose.position.z);
        group_jaco->setStartStateToCurrentState();  // current state of robot
	group_jaco->clearPoseTargets();
	group_jaco->setPlanningTime(10.0); // it was 10 before sherly
	group_jaco->setPoseTarget(jaco_pose); // Only working with KDL currently
	MOVEPLAN plan;
	bool success = group_jaco->plan(plan);  
		if(success) { // Ask if the visualized plan should be executed 
			//std::cout << "Want to use visualized plan to execute (y/N) ?" << std::endl;
			//std::string input;
			//std::cin >> input;
                         ros::Duration(1.0).sleep();
                         std::cout << "The plan is going to execute" << std::endl;

			//if (!std::strcmp(input.c_str(),"y")) {//Executing the pose 
				//group_jaco->execute(plan);   sherly you have to uncomment this later
                                jaco_driver_trajec_execute(plan.trajectory_.joint_trajectory);
                                ROS_INFO("hello i received yes to execute");
                                ros::Duration(5.0).sleep(); 
                           	if (moveit_controller_manager::ExecutionStatus::SUCCEEDED )
		                 {
			          return true;
		                 }    
			
               
		}
		else {
			ROS_ERROR("Couldn't plan movement to position (%.5lf, %.5lf, %.5lf) and orientation (%.5lf, %.5lf, %.5lf, %.5lf)",
					group_jaco->getPoseTarget().pose.position.x, group_jaco->getPoseTarget().pose.position.y, group_jaco->getPoseTarget().pose.position.z,
					group_jaco->getPoseTarget().pose.orientation.x, group_jaco->getPoseTarget().pose.orientation.y,
					group_jaco->getPoseTarget().pose.orientation.z, group_jaco->getPoseTarget().pose.orientation.w);
                     return false; 
		     }
         return false;
	}

bool jaco_grasp_function(const geometry_msgs::Pose::ConstPtr& jaco_pose, boost::shared_ptr<MOVEPI> group_jaco)
{
geometry_msgs::Pose abovePos1, abovePos2, p1, p2;
  
 double x0 = 5; 
 double x1 = -10;
 double y0 = -40; 
 double y1 = -60;
 double height = 20;
  p2 = jaco_random_poses(x0,x1,y0,y1,height);
  p1 = *jaco_pose;
  abovePos1 = p1;
  abovePos1.position.z = abovePos1.position.z + above_table;
  abovePos2 = p2;
  abovePos2.position.z = abovePos2.position.z + above_table;
 
  if (!jaco_move_to_position(abovePos1,group_jaco)) return false;
   keypressWait();
  if (!jaco_open_gripper()) return false;
  keypressWait();
  if (!jaco_move_to_position(p1,group_jaco)) return false;
  keypressWait();
  if (!jaco_close_gripper()) return false;
  keypressWait();
  if (!jaco_move_to_position(abovePos1,group_jaco)) return false;
  keypressWait();
  if (!jaco_move_to_position(abovePos2,group_jaco)) return false;
  keypressWait();
  if (!jaco_move_to_position(p2,group_jaco)) return false;
   keypressWait();
  if (!jaco_open_gripper()) return false;
  keypressWait();
  if (!jaco_move_to_position(abovePos2,group_jaco)) return false;

  return true;
}

void jaco_pose_callback(const geometry_msgs::Pose::ConstPtr& jaco_pose, boost::shared_ptr<MOVEPI> group_jaco) {
if(!grasp_on_progress)
 {
    grasp_on_progress = true;
   bool grasp_result =jaco_grasp_function(jaco_pose,group_jaco);
    if(grasp_result)
    {
      ROS_INFO("Execution of Grasp done");
     }else{
     ROS_WARN("Execution of Grasp was not able to complete");
     }
     grasp_on_progress=false;
     ros::Duration(3.0).sleep();
 }
}






//adding kinect as collision object
void add_collision_object(boost::shared_ptr<moveit::planning_interface::MoveGroup> group_jaco) {
	moveit::planning_interface::PlanningSceneInterface interface;
	ROS_INFO("Function call for add_collision object() gotten");
	std::vector<std::string> removed_objects;
	removed_objects.push_back("kinect");
	interface.removeCollisionObjects(removed_objects);
        removed_objects.push_back("pole");
        interface.removeCollisionObjects(removed_objects);
        removed_objects.push_back("youbot");
        interface.removeCollisionObjects(removed_objects);
	ROS_INFO("removal of collision objects executed");
         


	// Create kinect as CollisionObject for planning scene
	moveit_msgs::CollisionObject object1;
	object1.id = "kinect";
	object1.header.stamp = ros::Time::now();
	object1.header.frame_id = group_jaco->getPlanningFrame();//"jaco_link_base";
	ROS_INFO("Creating kinect object to planning scene with header frame_id %s", object1.header.frame_id.c_str());
	object1.operation = moveit_msgs::CollisionObject::ADD;
	// Create Shape for kinect
	shape_msgs::SolidPrimitive shape1;
	shape1.type = shape_msgs::SolidPrimitive::BOX;
	shape1.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	shape1.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.30;
	shape1.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.15;
	shape1.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.10;
	// Create Pose for kinect
	geometry_msgs::Pose pose1;
	pose1.position.x = -0.345129;
	pose1.position.y = -0.0803092;
	pose1.position.z = 0.527164;
	object1.primitives.clear();
	object1.primitives.push_back(shape1);
	object1.primitive_poses.clear();
	object1.primitive_poses.push_back(pose1);
	ROS_INFO("kinect object with pose and shape created");
        //************************************************************************************************
        // Create pole as CollisionObject for planning scene
	moveit_msgs::CollisionObject object2;
	object2.id = "pole";
	object2.header.stamp = ros::Time::now();
	object2.header.frame_id = group_jaco->getPlanningFrame();//"jaco_link_base";
	ROS_INFO("Creating pole object to planning scene with header frame_id %s", object2.header.frame_id.c_str());
	object2.operation = moveit_msgs::CollisionObject::ADD;
	// Create Shape for pole
	shape_msgs::SolidPrimitive shape2;
	shape2.type = shape_msgs::SolidPrimitive::BOX;
	shape2.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	shape2.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.10;
	shape2.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.10;
	shape2.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.48;
	// Create Pose for pole
	geometry_msgs::Pose pose2;
	pose2.position.x = -0.495129;
	pose2.position.y = -0.0203092;
	pose2.position.z = 0.0;
	object2.primitives.clear();
	object2.primitives.push_back(shape2);
	object2.primitive_poses.clear();
	object2.primitive_poses.push_back(pose2);
	ROS_INFO("pole object with pose and shape created");
         //************************************************************************************************
        // Create youbot as CollisionObject for planning scene
	moveit_msgs::CollisionObject object3;
	object3.id = "youbot";
	object3.header.stamp = ros::Time::now();
	object3.header.frame_id = group_jaco->getPlanningFrame();//"jaco_link_base";
	ROS_INFO("Creating youbot object to planning scene with header frame_id %s", object2.header.frame_id.c_str());
	object3.operation = moveit_msgs::CollisionObject::ADD;
	// Create Shape for youbot
	shape_msgs::SolidPrimitive shape3;
	shape3.type = shape_msgs::SolidPrimitive::BOX;
	shape3.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	shape3.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.60;
	shape3.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.40;
	shape3.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.15;
	// Create Pose for youbot
	geometry_msgs::Pose pose3;
	pose3.position.x = 0.0;//check these values
	pose3.position.y = 0.0;
	pose3.position.z = -0.15;
	object3.primitives.clear();
	object3.primitives.push_back(shape3);
	object3.primitive_poses.clear();
	object3.primitive_poses.push_back(pose3);
	ROS_INFO("youbot object with pose and shape created");

	// Create vector to publish objects to planning scene
	std::vector<moveit_msgs::CollisionObject> objects;
	objects.push_back(object1);
        objects.push_back(object2);
        objects.push_back(object3);
	interface.addCollisionObjects(objects);
	ROS_INFO("collision objects executed");
}


void gripper_callback(const std_msgs::String::ConstPtr &msg) {
	ROS_INFO("Gripper callback with message '%s' gotten", msg->data.c_str());
	gripper_ac->cancelAllGoals();
	if (!strcmp(msg->data.c_str(), "open")) {
		jaco_open_gripper();
	} else if (!strcmp(msg->data.c_str(), "close")) {
		jaco_close_gripper();
	       } else {
		      double position = std::atof(msg->data.c_str());
		          if (position < 0.0 || position > 1.0){
		          ROS_WARN("Unknown message for gripper callback: %s\n"
					"Allowed messages: 'open','close' and radian position [0,1]", msg->data.c_str());
		           }
	               }
}





//**************************************sherly commented the previous code***************************************************************//
/*

void jaco_pose_callback(const geometry_msgs::Pose::ConstPtr& jaco_pose, boost::shared_ptr<MOVEPI> group_jaco) {
	ROS_INFO("Callback to pick object from position (%.5lf, %.5lf, %.5lf) gotten!",
			jaco_pose->position.x, jaco_pose->position.y, jaco_pose->position.z);
        if(go_pick){
     
           jaco_open_gripper();
 
	group_jaco->setStartStateToCurrentState();  // current state of robot
	group_jaco->clearPoseTargets();
	group_jaco->setPlanningTime(30.0); // it was 10 before sherly
	group_jaco->setPoseTarget(*jaco_pose); // Only working with KDL currently
       
	//planning the pose
	MOVEPLAN plan;
        
	bool success = group_jaco->plan(plan);
           
	// Ask if the visualized plan should be executed 
		if (success) {
			ROS_INFO("Planning successfull!");
			std::cout << "Want to use visualized plan to execute (y/N) ?" << std::endl;
			std::string input;
			std::cin >> input;
			if (!std::strcmp(input.c_str(),"y")) {
	//Executing the pose 
                                go_pick = false;
				group_jaco->execute(plan);
				ROS_INFO("Execution done! Current pose:");
				//jaco_close_gripper();
				std::cout << group_jaco->getCurrentPose() << std::endl;
                              
                                jaco_close_gripper();
                            
                                ros::Duration(2.0).sleep();
				if(true)   //sherly change it for current reached
                                {
                                 group_jaco->setStartStateToCurrentState();  // current state of robot
	                         group_jaco->clearPoseTargets();
	                         group_jaco->setPlanningTime(10.0);
                                 double  x0 = 5; 
                                 double x1 = -10;
                                 double y0 = -40; 
                                 double y1 = -60;
                                 double height = 20;
                                 int positions = 1;
                                 bool success1;
                                 for (int count = 0; count < positions;count++)
                                 {
                                 Eigen::RowVectorXf random_pose(3);
            	                 random_pose = jaco_random_poses(x0,x1,y0,y1,height);
                                 geometry_msgs::Pose gpPose;
                                 gpPose.position.x=random_pose(0);
                                 gpPose.position.y=random_pose(1);
                                 gpPose.position.z=random_pose(2);
                                 gpPose.orientation.x=0;
                                 gpPose.orientation.y=0;
                                 gpPose.orientation.z=0;
                                 gpPose.orientation.w=1;
                                 group_jaco->setPoseTarget(gpPose);
                                 MOVEPLAN plan;
	                         success1 = group_jaco->plan(plan);
                                 if (success1) {
                                 //group_jaco->execute(plan);
                                 jaco_driver_trajec_execute(plan.trajectory_.joint_trajectory); 
                                 break;
                                 }
                                 }
				if(!success1){
                                ROS_INFO("OOPS!!!It seems like the pose cannot be planned :-(  Better luck next time :-p");
                                }
                                ros::Duration(1.0).sleep();
                                }else{
                                ROS_INFO("Looks like it was not able to grasp the object!!!!");
                                }
                                ros::Duration(2.0).sleep();
                                go_pick = true;
			}
		}
		else {
			ROS_ERROR("Couldn't plan movement to given pose with position (%.5lf, %.5lf, %.5lf) and orientation (%.5lf, %.5lf, %.5lf, %.5lf)",
					group_jaco->getPoseTarget().pose.position.x, group_jaco->getPoseTarget().pose.position.y, group_jaco->getPoseTarget().pose.position.z,
					group_jaco->getPoseTarget().pose.orientation.x, group_jaco->getPoseTarget().pose.orientation.y,
					group_jaco->getPoseTarget().pose.orientation.z, group_jaco->getPoseTarget().pose.orientation.w);
		}
             }
	}
*/
//****************************************sherly commented the previous code **********************************************************//
int main(int argc, char **argv) {
	init(argc, argv, "mrs_jaco_move");
	gripper_ac = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>("jaco_fingers_controller/gripper_action", true);
jaco_trajec_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("jaco_arm_controller/follow_joint_trajectory", true);
	
         // Spawn key press thread
        keypressThread = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&keypressLoop)));
	boost::shared_ptr<MOVEPI> group_jaco = boost::shared_ptr<MOVEPI>(new MOVEPI("jaco_arm"));
	// settings for move_group node
	group_jaco->setGoalTolerance(0.01);
	group_jaco->clearPoseTargets();
	group_jaco->setStartStateToCurrentState();
	//Initialize move_group node
	group_jaco->setEndEffector("jaco_gripper");
	group_jaco->setEndEffectorLink("jaco_gripper_toolframe");
	group_jaco->setPoseReferenceFrame("root");
	ROS_INFO(
			"jaco_move_arm initialized with end-effector link %s and name %s. Planning time: %.5lf",
			group_jaco->getEndEffectorLink().c_str(),
			group_jaco->getEndEffector().c_str(),
			group_jaco->getPlanningTime());
	ROS_INFO("Current pose:");
	std::cout << group_jaco->getCurrentPose() << std::endl;
        ROS_INFO("Calling function add_collision_object()");
	add_collision_object(group_jaco);
	ROS_INFO("Function call executed");
	ros::Duration(2.0).sleep();
	NodeHandle nh;
	//CallbackQueue movement_queue;

    Subscriber pick_sub =nh.subscribe<geometry_msgs::Pose>("jaco_poses_grasp", 1, boost::bind(jaco_pose_callback, _1, boost::cref(group_jaco)));
    Subscriber grip_sub =nh.subscribe<std_msgs::String>("gripper_status", 1, gripper_callback);
    AsyncSpinner spinner(2);
    spinner.start();
    spin();
    waitForShutdown();
    spinner.stop();
    return 0;
}

