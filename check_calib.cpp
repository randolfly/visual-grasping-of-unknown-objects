#include <ros/ros.h>
 #include <tf/transform_listener.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
//#include "std_msgs/String.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
int main(int argc, char** argv){
     ros::init(argc, argv, "check_calib");
   
     ros::NodeHandle node;
    
     tf::TransformListener listener;
 ros::Publisher gpTest = node.advertise<geometry_msgs::Pose>("/jaco_poses_move",1);
geometry_msgs::Pose gpPose;
ros::Rate rate(10.0);
  while (node.ok()){

  geometry_msgs::PointStamped kinect_point;
  kinect_point.header.frame_id = "camera_depth_optical_frame";

  //we'll just use the most recent transform available for our simple example
  kinect_point.header.stamp = ros::Time();

// hey the point is -0.3484,0.235486,0.78
// hey the point is -0.202157,0.232443,0.795
// hey the point is -0.326411,0.265893,0.676



  //just an arbitrary point in space
  kinect_point.point.x =  -0.326411;                 
  kinect_point.point.y = 0.265893;                 
  kinect_point.point.z =0.676;                  



  try{
    geometry_msgs::PointStamped jaco_point;
    listener.transformPoint("/root", kinect_point, jaco_point);

    ROS_INFO("kinect: (%.2f, %.2f. %.2f) -----> jaco: (%.2f, %.2f, %.2f) at time %.2f",
    		kinect_point.point.x, kinect_point.point.y, kinect_point.point.z,
			jaco_point.point.x, jaco_point.point.y, jaco_point.point.z, jaco_point.header.stamp.toSec());

gpPose.position.x=jaco_point.point.x;
gpPose.position.y=jaco_point.point.y;
gpPose.position.z=jaco_point.point.z;
gpPose.orientation.x=0;
gpPose.orientation.y=0;
gpPose.orientation.z=0;
gpPose.orientation.w=1;

 
        gpTest.publish(gpPose);
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"root\": %s", ex.what());
  }
 rate.sleep();
    }
return 0;
}

