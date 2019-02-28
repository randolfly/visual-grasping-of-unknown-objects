/*
 *
 *
 * Christina -TUW
 * 22.1.2014
 * earlier version: 26.03. 2012
 *
 * input:
 *
 *   pointcloud from topcamera read from topic /SS/camera/depth_registered/points
 *
 * output:
 *   pointcloud edited
 *
 *   output point cloud w.r.t. tf_frame /lwr on topic /SS/points2_object_in_rcs
 *
 *
 */

#include <pcl_ros/point_cloud.h>
#include <pc_merge_with_basket_rot.hpp>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"


CPCMerge::CPCMerge(ros::NodeHandle nh_)
{
  nh = nh_;
  pc_cam1_filled = false;
  //define publishers
  pc_for_basketdet_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/points2_object_in_rcs",1); //rcs:robot coordinate sys.

  pc_z_filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/points_after_z_filtering",1);
  pc_y_filter_pub =  nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/points_after_y_filtering",1);
  pc_x_filter_pub =  nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/points_after_x_filtering",1);
  pc_orig_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/point_cloud_orig",1);
  pc_sor_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/points_after_sor",1);

  position_highestpoint_pub = nh.advertise<std_msgs::String>("/SS/basket_position",1);
  //define subscriber
  pc_cam1_sub = nh.subscribe("/SS/camera/depth_registered/points",1, &CPCMerge::pc_cam1_callback, this);
}

void CPCMerge::pc_cam1_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_in)
{
  m.lock();

  ROS_INFO("pc_cam1 received");

  this->pc_cam1_filled = false;

  //search for tf transform for pc from cam1
  ros::Time now = ros::Time::now();
  bool foundTransform = tf_listener.waitForTransform("/root", "/camera_depth_optical_frame", now
		                                             /*(*pcl_in).header.stamp*/, ros::Duration(13.0));
  if (!foundTransform)
  {
	ROS_WARN("No pc_cam1 transform found");
	m.unlock();
	return;
  }

  //ROS_INFO(tf_listener);
  ROS_INFO("Transform pc_cam1: camera_depth_optical_frame to root found");
  pcl_ros::transformPointCloud("/root", *pcl_in, pc_cam1, tf_listener);
  this->pc_cam1_filled = true;

  //publishes point cloud
  publish_merged_pc();
  
  m.unlock();
}


void CPCMerge::publish_merged_pc()
{
	ROS_INFO("publish_merged_pc() started");
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_merged;
	pcl_cloud_merged = pc_cam1;

	//Filter Data (with basket detection and basket point elimination)
	filter_pc(pcl_cloud_merged, true); //last entry false <=> coordinates of highest points of scene are published instead of basket center point

    //publish manipulated and merged point cloud data (in root (not world!) coordinate system)
    pc_for_basketdet_out = pcl_cloud_merged;
    pc_for_basketdet_out.header.frame_id = "/root";
	pc_for_basketdet_pub.publish(pc_for_basketdet_out);

	ROS_INFO("point cloud of objects (rcs) and highest point position (of objects) were published");

        this->pc_cam1_filled = false;
}

// filter point cloud and cut off points outside a defined region
//if pub_basket_position == false than the position of the highest point is returned! otherwise the center of basket
void CPCMerge::filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged, bool pub_basket_position, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max )
{
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pcl_cloud_merged;
	pcl::PassThrough<pcl::PointXYZ> pass;
         
	ROS_INFO("Filtering outliers and cutting region");
	ROS_INFO("points before filtering: %d", pcl_cloud_merged.points.size());
         pc_orig_pub.publish(pcl_cloud_merged);
	//Filter w.r.t. axis z(follows right handed rule with thumb as z,middle finger as x and the other one as y)
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_min, z_max);// z filtering is for height (it removes lower(z_min) and upper(z_max) part of point cloud)
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_z);
	ROS_INFO("points after z filtering: %d", cloud_filtered_z->points.size());
        pc_z_filter_pub.publish(cloud_filtered_z);
	//Filter w.r.t. axis y
	pass.setInputCloud(cloud_filtered_z);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (y_min, y_max);
	pass.filter (*cloud_filtered_y);
	ROS_INFO("points after y filtering: %d", cloud_filtered_y->points.size());
        pc_y_filter_pub.publish(cloud_filtered_y);
	//Filter w.r.t. axis x
	pass.setInputCloud(cloud_filtered_y);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (x_min, x_max);
	pass.filter (*cloud_filtered_x);
	ROS_INFO("points after x filtering: %d", cloud_filtered_x->points.size());
        pc_x_filter_pub.publish(cloud_filtered_x);
	//Create the filtering object
        //Have to check what is StatisticalOutlierRemoval and how it works
     
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_filtered_x);
	sor.setMeanK(50);
	sor.setStddevMulThresh (1.0);
	sor.filter(pcl_cloud_merged);
        pc_sor_pub.publish(pcl_cloud_merged);

	if (pub_basket_position == false){ // than the position of the highest point is published!
	    ROS_INFO("%d",pcl_cloud_merged.points.size());
		float z_max = -1000;			//max heigt val for z
		float x_for_z_max = -1000;		//coordinates for x,y at position of maximal z
		float y_for_z_max = -1000;
	    for (unsigned int i = 0; i < pcl_cloud_merged.points.size(); ++i)
		{
		  if (pcl_cloud_merged.points[i].z > z_max)
		  {
			z_max = pcl_cloud_merged.points[i].z;
			x_for_z_max = pcl_cloud_merged.points[i].x;
			y_for_z_max = pcl_cloud_merged.points[i].y;
		  }
		}
	    //Publish coordinates of heighest points ("basket center" if there is no basket)
		std_msgs::String msgStrCorners;
		std::stringstream ss;
                float x_box_center = -33;
                float y_box_center = -68;

		//ss << x_for_z_max << " "  << y_for_z_max << " " << 0; original sherly
                ss << x_box_center << " "  << y_box_center  << " " << 0; // changed sherly
		msgStrCorners.data = ss.str();
		position_highestpoint_pub.publish(msgStrCorners);
	} else {
		basket_detection<pcl::PointXYZ>(pcl_cloud_merged,pub_basket_position);
	}
}



int main (int argc, char** argv)
{
  ROS_INFO("ROS NODE pc_merge_1cam_nobox started (point_cloud_edit)");
  ros::init(argc, argv, "point_cloud_edit");
  ros::NodeHandle nh;
  CPCMerge * pc_merge = new CPCMerge(nh);

  ros::spin();
  return (0);
}
