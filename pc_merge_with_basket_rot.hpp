/*
 * pc_merge_with_basket_rot.hpp
 *
 *  Created on: Dec 29, 2011
 *      Author: grasp
 */

#ifndef PC_MERGE_WITH_BASKET_ROT_HPP_
#define PC_MERGE_WITH_BASKET_ROT_HPP_
#define SOFTSTUFFHEIGHT 0.035		//david, new 30.1.2012 => 1.5 cm for inaccurace

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/io/io.h"
#include "pcl_ros/publisher.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/thread/mutex.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <sstream>

using namespace std;

class CPCMerge
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	bool pc_cam1_filled, pc_cam2_filled;
	bool use_two_cams;
	ros::Publisher pc_merged_pub;
	ros::Publisher nr_segmented_pc_pub;
	ros::Publisher pc_for_basketdet_pub;
	ros::Publisher pc_for_basketdet_cam1_pub;		//in camera coordinate system, basket points eliminated
	ros::Publisher position_highestpoint_pub;
        ros::Publisher pc_z_filter_pub;
        ros::Publisher pc_y_filter_pub;
        ros::Publisher pc_x_filter_pub;
        ros::Publisher pc_orig_pub; 
        ros::Publisher pc_sor_pub;

	ros::Subscriber pc_cam1_sub;
	ros::Subscriber pc_cam1_sub2;
	ros::Subscriber pc_cam2_sub;
	pcl::PointCloud<pcl::PointXYZ> pc_cam1;
	pcl::PointCloud<pcl::PointXYZ> pc_cam2;
	pcl::PointCloud<pcl::PointXYZ> pc_for_basketdet_out;
  	ros::NodeHandle nh;
  	boost::mutex m;

	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_cam1_without_basket;

	tf::TransformListener tf_listener;

	CPCMerge(ros::NodeHandle nh_);

	// merges point clouds from 2 different cameras
	void publish_merged_pc();
	// filters outliers and points outside defined region
	void filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged,
			      bool pub_basket_position=false, float x_min=-0.95, float x_max=0.90,
			       float y_min=-0.95, float y_max=0.05, float z_min=0.12, float z_max = 0.95);// in terms of root coordinate system

//***********************************************************************************************************************************
// filters outliers and points outside defined region
//	void filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged,
//			       bool pub_basket_position=true, float x_min=0.25, float x_max=-0.47,
//			       float y_min=-0.70, float y_max=0.05, float z_min=0.55, float z_max = 0.95);//sherly value in terms of kinect coordinate system (z pointing in front,x pointing in the left side, y pointing upward )
//*************************************************************************************************************************************


	void segment_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_merged );
	// receives point clouds from camera #1
	void pc_cam1_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_in);

	// basket detection
	template <typename PointT>
	void basket_detection(pcl::PointCloud<PointT>& pcl_cloud_merged,
			              bool pub_basket_position);
};

template <typename PointT>
void CPCMerge::basket_detection(pcl::PointCloud<PointT>& pcl_cloud_merged, //sherly change the code for box center detection
		                        bool pub_basket_position)
{
	pcl::PointCloud<PointT> pc_for_basketdet;
	pcl::PointCloud<PointT> pc_for_basketdet2;
	pcl::PointCloud<PointT> pc_for_basketdet3;
	pcl::PointCloud<PointT> pc_for_basketdet_help;
	pcl::PointCloud<PointT> final;

	pcl::PointCloud<PointT> cloud_tmp_rotated;
    pcl::PointCloud<PointT> cloud_out;

    pc_for_basketdet.header.frame_id = pcl_cloud_merged.header.frame_id;
    pc_for_basketdet2.header.frame_id = pcl_cloud_merged.header.frame_id;
    pc_for_basketdet3.header.frame_id = pcl_cloud_merged.header.frame_id;
    pc_for_basketdet_help.header.frame_id = pcl_cloud_merged.header.frame_id;

	//BASKET DETECTION and box deletion

	//step_1
	//filter by height
    ROS_INFO("%d",pcl_cloud_merged.points.size());
	for (unsigned int i = 0; i < pcl_cloud_merged.points.size(); ++i)
	{
	  if (pcl_cloud_merged.points[i].z < 0.17)// height of table wrt root coordinate system
	  {
		PointT pnt_tmp = pcl_cloud_merged.points[i];
		pc_for_basketdet.push_back(pnt_tmp);
	  }
	}
	ROS_INFO("%d",pc_for_basketdet.points.size());
	//step_2
	//find max/min for x/y
	double x_mi = 1000;
	double x_ma = -1000;
	double y_mi = 1000;
	double y_ma = -1000;
	double z_ma = -1000;
	for (unsigned int i = 0; i < pc_for_basketdet.points.size(); ++i)
	{
	  if (pc_for_basketdet.points[i].x > x_ma)
		x_ma = pc_for_basketdet.points[i].x;
	  if (pc_for_basketdet.points[i].x < x_mi)
		x_mi = pc_for_basketdet.points[i].x;
	  if (pc_for_basketdet.points[i].y > y_ma)
		y_ma = pc_for_basketdet.points[i].y;
	  if (pc_for_basketdet.points[i].y < y_mi)
	    y_mi = pc_for_basketdet.points[i].y;

	  if (pc_for_basketdet.points[i].z > z_ma)
		z_ma = pc_for_basketdet.points[i].z;
	}

	//step_3
	//define approximate center from step_2
	double box_center_x = (x_ma + x_mi)/2;
	double box_center_y  = (y_ma + y_mi)/2;
	
	  double alpha= 0;//Assuming angle as zero

	  ROS_INFO("box_center_x and box_center_y (genau): %f %f",box_center_x,box_center_y);

	 

	if (pub_basket_position)
	{
	  //Publish corner points of basket
	  std_msgs::String msgStrCorners;
	  std::stringstream ss;
	  ss << box_center_x << " "  << box_center_y << " " << alpha;
	  msgStrCorners.data = ss.str();
	  position_highestpoint_pub.publish(msgStrCorners);
	}

	//END BASKET DETECTION
}

#endif /* PC_MERGE_WITH_BASKET_ROT_HPP_ */
