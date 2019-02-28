/////////////////////////////////////////////// INCLUDES

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <cstdlib>
#include <Eigen/Geometry>
#include <cstring>
//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
//#include "std_msgs/String.h"
#include <ros/callback_queue.h>
//moveit packages

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/Constraints.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include <geometric_shapes/shape_operations.h>

//boost bind to use multiple arguments
#include "boost/bind.hpp"
#include "boost/shared_ptr.hpp"
#include "boost/ref.hpp"



using namespace ros;
typedef moveit::planning_interface::MoveGroup MOVEPI;
typedef moveit::planning_interface::MoveGroup::Plan  MOVEPLAN;
//typedef moveit::planning_interface::PlanningSceneInterface scene_plan;




/*
void add_test_object() {
	ROS_INFO("TEST FUNCTION add_test_object EXECUTED");
	ros::NodeHandle nh("jaco_move_and_pick");
	ros::Publisher table_pub = nh.advertise<moveit_msgs::CollisionObject>("object", 1000);

	ROS_INFO("Attempting to add test object to planning scene");
	// Remove table if already exists
	ROS_INFO("Removing test object from planning scene interface");
	//scene_interface.removeCollisionObjects(remove_objects);
	ros::Duration(2.0).sleep();
	// Create table as CollisionObject for planning scene
	moveit_msgs::CollisionObject object;
	object.id = "testobject";
	object.header.stamp = ros::Time::now();
	object.header.frame_id = "root";
	object.operation = moveit_msgs::CollisionObject::ADD;
	// Create Shape for test object
	shape_msgs::SolidPrimitive shape;
	shape.type = shape_msgs::SolidPrimitive::BOX;
	shape.dimensions.resize(10);
	shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.01;
	shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.20;
	shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.30;
	// Create Pose for test object
	geometry_msgs::Pose pose;
	pose.position.x = -0.1;
	pose.position.y = 0.5;
	pose.position.z = 0.15;
	object.primitives.clear();
	object.primitives.push_back(shape);
	object.primitive_poses.clear();
	object.primitive_poses.push_back(pose);
	ROS_INFO("Test object with pose and shape created");
	// Create vector to publish objects to planning scene

	table_pub.publish(object);
	ros::Duration(3.0).sleep();
	table_pub.shutdown();

	ROS_INFO("TEST FUNCTION add_test_object FINISHED");
}
*/

	

void jaco_pose_callback(const geometry_msgs::Pose::ConstPtr& jaco_pose, boost::shared_ptr<MOVEPI> group_jaco) {
	ROS_INFO("Callback to move arm to position (%.5lf, %.5lf, %.5lf) gotten!",
			jaco_pose->position.x, jaco_pose->position.y, jaco_pose->position.z);

	geometry_msgs::Pose pickuppose = *jaco_pose;

		geometry_msgs::Quaternion q = pickuppose.orientation;
		Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
		quat.normalize();
		Eigen::Quaterniond::Vector3 vec(0, 0, -1);
		Eigen::Quaterniond::Vector3 tvec = quat._transformVector(vec);
		ROS_INFO("Eigen Quaternion (place) (x,y,z,w): %f,es%f,%f,%f", quat.x(),quat.y(),quat.z(),quat.w());
		ROS_INFO("Eigen Rotated vector (place): %f,%f,%f", tvec[0],tvec[1],tvec[2]);
		const double PICK_DIST = 0.1;
		//pickuppose.position.x -= PICK_DIST * tvec[0];
		//pickuppose.position.y -= PICK_DIST * tvec[1];
		//pickuppose.position.z -= PICK_DIST * tvec[2];
		//pickuppose.orientation.x =quat.x();
		//pickuppose.orientation.y =quat.y();
		//pickuppose.orientation.z =quat.z();
		//pickuppose.orientation.w =quat.w();


	group_jaco->setStartStateToCurrentState();  // current state of robot
	group_jaco->clearPoseTargets();
	group_jaco->setPlanningTime(15.0);
	//group_jaco->setPoseTarget(*jaco_pose); // Only working with KDL currently



	 robot_state::RobotState start_state(*group_jaco->getCurrentState());

	group_jaco->setPoseTarget(pickuppose); // Only working with KDL currently

	ROS_INFO(" plan movement to given pose with position (%.5lf, %.5lf, %.5lf) and orientation (%.5lf, %.5lf, %.5lf, %.5lf)",
								group_jaco->getPoseTarget().pose.position.x, group_jaco->getPoseTarget().pose.position.y, group_jaco->getPoseTarget().pose.position.z,
								group_jaco->getPoseTarget().pose.orientation.x, group_jaco->getPoseTarget().pose.orientation.y,
								group_jaco->getPoseTarget().pose.orientation.z, group_jaco->getPoseTarget().pose.orientation.w);

	ROS_INFO("Reference frame: %s", group_jaco->getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", group_jaco->getEndEffectorLink().c_str());

	//group_jaco->setOrientationTarget(0.1,0.1,0.1,0.1,"jaco_gripper_toolframe");
	//group_jaco->setRPYTarget(0.1,0.2,0.3,"jaco_gripper_toolframe");
	//	ROS_INFO("oooooooooooooooooooooooooooooooooooooooooooooooommmmmmmmmmmmmmmmyyyyyyyyyyyyyyyyyyggggggggggg");
	//	group_jaco->setRPYTarget(0.1,0.2,0.3,"jaco_link_hand");
	//	ROS_INFO("rrrrrrrrrrrrrraaaaaaaaaaajjjjjjjjjjjjjjjjjjjjjjjjjjj");


/*
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//group_jaco->attachObject("bluemmfinalreg_final.ply");

	//scene_plan::addCollisionObjects()
// attach mesh object:: this is for mrs jaco object grasp plan

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = group_jaco->getPlanningFrame();

	// The id of the object is used to identify it.
	collision_object.id = "Blue_Chair";
/*
	// A pose for the mesh (specified relative to frame_id)
		geometry_msgs::Pose mesh_pose;
		mesh_pose.orientation.w = 1.0;
		mesh_pose.position.x =  0.6;
		mesh_pose.position.y = -0.4;
		mesh_pose.position.z =  1.2;

	shapes::Mesh* mesh= shapes::createMeshFromResource("bluemmfinalreg_final.stl");
	shape_msgs::Mesh ply_mesh;
	shapes::ShapeMsg ply_mesh_msg = ply_mesh;
	shapes::constructMsgFromShape(mesh,ply_mesh_msg);
	//collision_object.meshes.push_back(ply_mesh);
	//collision_object.mesh_poses.push_back(mesh_pose);
	collision_object.meshes[0]= ply_mesh;
	collision_object.mesh_poses[0]= mesh_pose;
	collision_object.operation = collision_object.ADD;

*/
/*
	// Define a box to add to the world.
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(10);
	primitive.dimensions[0] = 0.4;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.4;


	// A pose for the box (specified relative to frame_id)
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0;
	box_pose.position.y = 0;
	box_pose.position.z =  0;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_object.header.frame_id = "root";
	collision_object.header.stamp = ros::Time::now();
	collision_objects.push_back(collision_object);


	ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

	//Sleep so we have time to see the object in RViz
	sleep(2.0);
	ros::Publisher chair_pub = nh.advertise<moveit_msgs::CollisionObject>("chair_object", 1000);
	chair_pub.publish(object);
		ros::Duration(3.0).sleep();
		chair_pub.shutdown();
		*/
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	//planning the pose
	MOVEPLAN plan;

	bool success = group_jaco->plan(plan);

	/* Ask if the visualized plan should be executed */
		if (success) {

			ROS_INFO("Planning successfull!");

			std::cout << "Want to use visualized plan to execute (y/N) ?" << std::endl;
			std::string input;
			std::cin >> input;
			if (!std::strcmp(input.c_str(),"y"))
			{
	//Executing the pose
				group_jaco->execute(plan);
				ROS_INFO("Execution done! Current pose:");
				ROS_INFO("Execution done! orientation:");
				ros::Duration(5.0).sleep();

						ROS_INFO(" got current pose (%.5lf, %.5lf, %.5lf) and orientation (%.5lf, %.5lf, %.5lf, %.5lf)",
													group_jaco->getCurrentPose().pose.position.x, group_jaco->getCurrentPose().pose.position.y, group_jaco->getCurrentPose().pose.position.z,
													group_jaco->getCurrentPose().pose.orientation.x, group_jaco->getCurrentPose().pose.orientation.y,
													group_jaco->getCurrentPose().pose.orientation.z, group_jaco->getCurrentPose().pose.orientation.w);

				std::cout << group_jaco->getCurrentPose() << std::endl;
				std::cout << group_jaco->getEndEffector() << std::endl;
				std::cout << group_jaco->getEndEffectorLink() << std::endl;
                                
                     


			}
/*
			group_jaco->execute(plan);
							ROS_INFO("Execution done! Current pose:");
							ROS_INFO("Execution done! orientation:");

			group_jaco->execute(plan);
			ROS_INFO("Execution done! Current pose:");
			std::cout << group_jaco->getCurrentPose() << std::endl;
*/
		}
		else {
			ROS_ERROR("Couldn't plan movement to given pose with position (%.5lf, %.5lf, %.5lf) and orientation (%.5lf, %.5lf, %.5lf, %.5lf)",
					group_jaco->getPoseTarget().pose.position.x, group_jaco->getPoseTarget().pose.position.y, group_jaco->getPoseTarget().pose.position.z,
					group_jaco->getPoseTarget().pose.orientation.x, group_jaco->getPoseTarget().pose.orientation.y,
					group_jaco->getPoseTarget().pose.orientation.z, group_jaco->getPoseTarget().pose.orientation.w);
		}


		//ros::Publisher raj_pub = nh.advertise<geometry_msgs::Pose>("jaco_poses",1000);
		//raj_pub.publish(group_jaco->getVariableCount());
	}

int main(int argc, char **argv) {
	init(argc, argv, "mrs_jaco_move");
	NodeHandle nh;

	//////////////////////////////////////////////////////////////////////
	    // matrix to queternions
	    //static tf::TransformBroadcaster br;
	    Eigen::Matrix4f Tm;

	//robot to camera
	
	    Tm <<    -0.999639, -0.0120515,  0.0240002,  -0.460978,
-0.0265387,   0.580261,  -0.813998, -0.0424326,
-0.00411647,  -0.814342,  -0.580371,   0.552427,
         0 ,         0 ,        -0 ,         1;





          //        -0.886114,  -0.283613,   0.366559,  -0.703007,
	//	    -0.462642,   0.494114,   -0.73608, -0.0880144,
 	//	     0.0276405,  -0.821836,  -0.569053,   0.571888,
         //		0,          0,          0,          1;


	    /*
	    Eigen::Matrix3f rotation = Tm.block<3, 3>(0, 0);

	    Eigen::Vector3f translation = Tm.block<3, 1>(0, 3);

		std::cout << "Transformation matrix:" << std::endl << std::endl;
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		std::cout << std::endl;
		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

	*/



	    tf::Vector3 origin;
	    origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
	    ROS_INFO("Translation:");
	    std::cout << origin;



	    tf::Matrix3x3 tf3d;
	    tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
	          static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
	          static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));
		ROS_INFO("rotaation to queternion:");

	    tf::Quaternion tfqt;
	    tf3d.transpose().getRotation(tfqt);
	    tf::Quaternion ctr;
	    tf3d.getRotation(ctr);
	    std::cout << tfqt;


	                    		ROS_INFO("queternion values:");

	                    		printf("\t\tt = < %0.6f, %0.6f, %0.6f %0.6f>\n", tfqt.x(), tfqt.y(), tfqt.z(), tfqt.w());
	                    		ROS_INFO("queternion values:");
	                    		printf("ctr = < %0.6f, %0.6f, %0.6f %0.6f>\n", ctr.x(), ctr.y(), ctr.z(), ctr.w());
	    //tf::Transform transform;
	    //transform.setOrigin(origin);
	   // transform.setRotation(tfqt);
	   // while (true) br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "camera_link"));

	    ///////////////////////////////////////////////////









	boost::shared_ptr<MOVEPI> group_jaco = boost::shared_ptr<MOVEPI>(
				new MOVEPI("jaco_arm"));

	//jaco_arm_gripframe_mrs
	//jaco_move_arm
	// settings for move_group node
	group_jaco->setGoalTolerance(0.01);
	group_jaco->clearPoseTargets();
	group_jaco->setStartStateToCurrentState();

	//Initialize move_group node
	//group_jaco->setEndEffector("jaco_gripper_gripframe_mrs");
	//group_jaco->setEndEffectorLink("jaco_gripper_toolframe");
	//group_jaco->setPoseReferenceFrame("root");
	//group_jaco->setEndEffector("jaco_link_hand");
	group_jaco->setEndEffector("jaco_gripper");
	group_jaco->setEndEffectorLink("jaco_gripper_toolframe");
	group_jaco->setPoseReferenceFrame("root");
	ROS_INFO(
			"jaco_move_arm initialized with end-effector link %s and name %s. Planning time: %.5lf",
			group_jaco->getEndEffectorLink().c_str(),
			group_jaco->getEndEffector().c_str(),
			group_jaco->getPlanningTime());
	ROS_INFO("Current pose:");

		std::cout <<  group_jaco->getCurrentState() << std::endl;
	std::cout << group_jaco->getCurrentPose() << std::endl;

	//CallbackQueue movement_queue;
	//CallbackQueue movement_queue;
		//SubscribeOptions options = SubscribeOptions::create<geometry_msgs::Pose>("jaco_poses_move", 1000, boost::bind(jaco_pose_callback, _1, group_jaco), VoidPtr(), &movement_queue);
		//Subscriber pose_sub = nh.subscribe(options);
	//add_test_object();
	ROS_INFO("TEST FUNCTION add_test_object EXECUTED");
	//ros::NodeHandle nh("jaco_move_and_pick");


	/*ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
	  ros::WallDuration sleep_t(0.5);
	  sleep_t.sleep();
	}
	ROS_INFO("Attempting to add test object to planning scene");
	// Remove table if already exists
	ROS_INFO("Removing test object from planning scene interface");
	//scene_interface.removeCollisionObjects(remove_objects);
	ros::Duration(2.0).sleep();



*/





	//object.header.frame_id = group_jaco->getPlanningFrame();

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*

ros::Publisher bench_pub = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
ros::Duration(0.5).sleep();
	// Create table as CollisionObject for planning scene
	    moveit_msgs::CollisionObject bench;


	// Create Shape and for test object
	shape_msgs::SolidPrimitive shape;
	shape.type = shape_msgs::SolidPrimitive::BOX;
	shape.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.35;
	shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.35;
	shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
	// Create Pose for test object
	geometry_msgs::Pose pose;
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = -0.099;
	bench.header.frame_id = "/root";
	bench.header.stamp = ros::Time::now();
	bench.primitives.push_back(shape);
	bench.primitive_poses.push_back(pose);
	bench.operation = bench.ADD;
	bench.id = "bench_object";
	bench_pub.publish(bench);
	ros::Duration(3.0).sleep();

	ROS_INFO("Test object with pose and shape created");

	moveit::planning_interface::PlanningSceneInterface scene_interface;

	// Create vector to publish objects to planning scene
		std::vector<moveit_msgs::CollisionObject> benches;
	// Create vector to publish objects to planning scene
		benches.push_back(bench);
	scene_interface.addCollisionObjects(benches);
	std::vector<std::string> benches_ids;
	benches_ids.push_back(bench.id);
	scene_interface.getObjectPoses(benches_ids);
	ros::Duration(2.0).sleep();
	ROS_INFO("chair addition executed");

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ros::Publisher chair_pub = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
ros::Duration(0.5).sleep();
	// Create table as CollisionObject for planning scene
	    moveit_msgs::CollisionObject object;

		ROS_INFO("mesh 1");







	shapes::Mesh* obj_mesh;
	obj_mesh= shapes::createMeshFromResource("file:///home/raj/catkin_ws/src/jaco_description/meshes/bluemmfinalreg_final.dae");
	shapes::ShapeConstPtr shape_pointer(obj_mesh);
    std::vector<shapes::ShapeConstPtr> shapes_vector;
    shapes_vector.push_back(shape_pointer);
    shapes::ShapeMsg dae_mesh_msg;
            shapes::constructMsgFromShape(obj_mesh, dae_mesh_msg);
            shape_msgs::Mesh dae_mesh = boost::get<shape_msgs::Mesh>(dae_mesh_msg);


	ROS_INFO("mesh 2");

	// Create Pose for test object
	// A pose for the mesh (specified relative to frame_id)
	//      specify pose of bowl
	        geometry_msgs::Pose mesh_pose;
	        mesh_pose.position.x = -0.569;
	        mesh_pose.position.y = -0.143;
	        mesh_pose.position.z = 0.219;
	        mesh_pose.orientation.x = -0.602;
	        mesh_pose.orientation.y = 0.150;
	        mesh_pose.orientation.z = 0.777;
	        mesh_pose.orientation.w = 0.105;

	object.header.frame_id = "/root";
	object.header.stamp = ros::Time::now();
	//object.primitives.push_back(shape);
	//object.primitive_poses.push_back(pose);
	object.meshes.push_back(dae_mesh);
	object.mesh_poses.push_back(mesh_pose);
	object.operation = object.ADD;
	object.id = "chair_object";
	chair_pub.publish(object);
	ros::Duration(3.0).sleep();
	ROS_INFO("mesh 3");
	ROS_INFO("Test object with pose and shape created");

	//moveit::planning_interface::PlanningSceneInterface scene_interface;

	// Create vector to publish objects to planning scene
		std::vector<moveit_msgs::CollisionObject> objects;
	// Create vector to publish objects to planning scene
	objects.push_back(object);
	scene_interface.addCollisionObjects(objects);
	std::vector<std::string> object_ids;
	object_ids.push_back(object.id);
	scene_interface.getObjectPoses(object_ids);
	ros::Duration(2.0).sleep();
	ROS_INFO("chair addition executed");

	ROS_INFO("mesh 3");


	//collision_object.meshes.push_back(ply_mesh);
	//collision_object.mesh_poses.push_back(mesh_pose);



*/




	//table_pub.shutdown();
	/*
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(object);
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene);
	*/
	ROS_INFO("TEST FUNCTION add_test_object FINISHED");

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Subscriber raj_sub =nh.subscribe<geometry_msgs::Pose>("jaco_poses_move", 1, boost::bind(jaco_pose_callback, _1, boost::cref(group_jaco)));












    //AsyncSpinner spinner(0, &movement_queue);
    AsyncSpinner spinner(1);
	spinner.start();
    	//ros::Rate loop_rate(10);
/*



    			geometry_msgs::Pose jaco_pose;

    					jaco_pose.orientation.x = 0.0;
    					jaco_pose.orientation.y = 0.0;
    					jaco_pose.orientation.z = 0.0;
    					jaco_pose.orientation.w = 1.0;
    					jaco_pose.position.x = -0.15;
    					jaco_pose.position.y = -0.2;
    					jaco_pose.position.z = 0.3;
    					ROS_INFO("Custom position given at (%.5lf,%.5lf,%.5lf)",
    							jaco_pose.position.x, jaco_pose.position.y, jaco_pose.position.z);
    				    ros::Publisher raj_pub = nh.advertise<geometry_msgs::Pose>("jaco_poses_move",1000);

    	ROS_INFO("Position (%.5lf, %.5lf, %.5lf) is to be published",jaco_pose.position.x,jaco_pose.position.y,jaco_pose.position.z);
    	raj_pub.publish(jaco_pose);
    		ROS_INFO("Position (%.5lf, %.5lf, %.5lf) has been published",jaco_pose.position.x,jaco_pose.position.y,jaco_pose.position.z);
 */

    		spin();

    			waitForShutdown();
    			spinner.stop();

    			return 0;
}

