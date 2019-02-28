/*
 * pca_grasping.cpp
 *

 *
 *     Program is calculating grasp points  from a point cloud.
 *     In a first step the point cloud is read from a ROS topic and a heightsgrid is created
 *     (for rotated and tilted pointclouds <==> different rolls of hand and approaching values).
 *     For each 14x14 square of the hightsgrid a featurevector is created.
 *     Using SVM with an existing model file, it is predicted if the center of the square is a good
 *     grasping point. For good grasping points the coordinates and the direction of the approach vectors
 *     are published.
 *
 *      == Input ==
 *      A point cloud from objects inside a box
 *      (points of box are eliminated)
 *
 *      == Output ==
 *      Grasp points  which are detected using PCA
 *
 *
 *      USAGE
 *
 *      PARAMETERS
 *
 *      outputpath_full = "/tmp/features.txt";
 *      path_svm_output = "/tmp/output_calc_gp.txt";
 *
 */


//System includes
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <Eigen/Dense>
#include </home/christina/catkin_ws/src/pca_grasping/include/pca_grasping.h>
//#include <util.h>


//PCL includes
#include "/usr/include/pcl-1.7/pcl/io/io.h"
#include "pcl/point_cloud.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl_ros/publisher.h"
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/pca.h>
#include <pcl/common/time.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//opencv includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// Boost
#include <boost/foreach.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
//ROS includes
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#define PI 3.141592653

typedef pcl::PointXYZ PointT;

using namespace std;
using namespace cv;


void pca_grasping::closestPoint(const pcl::PointCloud< PointT >::Ptr cloud,
		  const pcl::KdTreeFLANN< PointT > &kdtree,
		  const Eigen::Vector3f &p1,
		  Eigen::Vector3f &p2)
{
  std::vector< int > index(1);
  std::vector< float > dist(1);

  PointT t1; t1.x = p1(0); t1.y = p1(1); t1.z = p1(2);
  int res = kdtree.nearestKSearch (t1, 1, index, dist); assert(res != 0);
  PointT tmp = (*cloud).points[index[0]];
  p2(0) = tmp.x;
  p2(1) = tmp.y;
  p2(2) = tmp.z;
}

//only thing which is done here: read point cloud and do GP calculation using PCA
void pca_grasping::read_pc_cb(const sensor_msgs::PointCloud2ConstPtr& pc_in)
{
	ROS_INFO("\nFrom grasping_pca: point cloud received");

	pcl::PointCloud<PointT>::Ptr cloud = 
		boost::shared_ptr< pcl::PointCloud< PointT > > 
      		(new pcl::PointCloud< PointT > ());

	pcl::fromROSMsg(*pc_in, *cloud);

	//transform point cloud to PCL
	struct GraspPCA grasp;
	pca_grasping::getGraspPCA(cloud, grasp);
	
}




void pca_grasping::getGraspPCA(pcl::PointCloud<PointT>::Ptr cloud,
		      struct GraspPCA &grasp){
  grasp.cloud_all = cloud;
  grasp_safety_offset = 0.01f; //1cm
  grasp_max_width = 0.1f; //10cm

#if 0
  plane_v[0] =0.0; //- 0.000139633	;//0.0f;
 plane_v[1] = 0.0;//0.01553985; //0.0f;
  plane_v[2] = -0.0165;//0.00451393932;//-1.0f; 
plane_v[3] = 0.001485; //-0.0072289379239;//0.0f;
#endif 
 plane(0) = 0.0; //- 0.000139633	;//0.0f;
 plane(1) = 0.0;//0.01553985; //0.0f;
 plane(2) = -1.0;//0.00451393932;//-1.0f; 
 plane(3) = 0.00; //-0.0072289379239;//0.0f;

 // these values are in robot coordinate system
 std::vector < float > plane_v(4);
 plane_v[0] = plane(0); plane_v[1] = plane(1);
 plane_v[2] = plane(2); plane_v[3] = plane(3);

  finger_axis(0) = 0.0f;
  finger_axis(1) = 1.0f;
  finger_axis(2) = 0.0f;

  finger_axis_orthogonal(0) =1.0f;
  finger_axis_orthogonal(1) =0.0f;
  finger_axis_orthogonal(2) =0.0f;

  // Create a planar coefficient model and a projection onto the wrist plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values = plane_v; 
  grasp.proj.setModelType (pcl::SACMODEL_PLANE);
  grasp.proj.setModelCoefficients (coefficients);

  // Create KD-tree for minimum distance computation
  pcl::KdTreeFLANN< PointT > kdtree;
  kdtree.setInputCloud (cloud);

  // 1. Project points onto a plane that goes through robot hand wrist
  //    (note that the plane needs to be in world coordinates)
  // 2. Compute the first two dominating vectors using PCA
  // 3. Use the narrower vector for choosing the grasp points


  // 1. Project points onto a plane that is orthogonal to the robot
  //    hand axis. Because points are in Kinect coordinates, the plane
  //    needs to be in Kinect coordinates, that is, projected from
  //    robot coordinates to Kinect coordinates.

  // Create a planar coefficient model and project the points onto the plane
  pcl::PointCloud<PointT> proj_cloud;
  grasp.proj.setInputCloud (cloud);
  grasp.proj.filter (proj_cloud);

  // Make density equal in all parts of the 2D pointcloud (projected
  // 3D cloud) by sampling voxels of 2*radius equal to the largest
  // distance to a neighboring point.

  // Find maximum minimum distance using a KD-tree
  pcl::KdTreeFLANN< PointT > kd(true);
  kd.setInputCloud (proj_cloud.makeShared());
  float maxDist = 0.0f;
  BOOST_FOREACH(const PointT &p, proj_cloud.points)
  {
    std::vector< int > index(2); std::vector< float > dist(2);
    int res = kd.nearestKSearch (p, 2, index, dist); assert(res != 0);
    float d = sqrt(dist[1]);
    if ((d < maxMaxDist) && (d > maxDist))
    {
      maxDist = d;
    }
  }
  if (maxDist == 0.0f) maxDist = minMaxDist;

  // Downsample 2D cloud
  pcl::UniformSampling< PointT > us;
  us.setInputCloud (proj_cloud.makeShared());
  us.setRadiusSearch (maxDist / 2);
  pcl::PointCloud<int> sampled_indices;
  us.compute (sampled_indices);
  pcl::copyPointCloud (proj_cloud, sampled_indices.points, *grasp.cloud2D);

  // 2. Compute the dominating vectors 'eigen_vectors' using PCA for
  // the points projected onto the plane
  pcl::PCA<PointT> pca(true);
  pca.setInputCloud(grasp.cloud2D);
  Eigen::Vector3f vector = pca.getEigenVectors().col(1);

#if 0
  // TEST
  {
    Eigen::Vector3f v0 = pca.getEigenVectors().col(0);
    Eigen::Vector3f v1 = vector;
    //Eigen::Vector3f vr0;
    //Eigen::Vector3f vr1;
    //JacoClientUtil jaco;
    //jaco.rotateWorld(v0, vr0);
    //jaco.rotateWorld(v1, vr1);
    std::cout << "PCA world eigen vectors: " << std::endl <<
      "  Vector 0:" << v0 << std::endl <<
      "  Vector 1:" << v1 << std::endl;
    //std::cout << "PCA robot eigen vectors: " << std::endl <<
    //  "  Vector 0:" << vr0 << std::endl <<
    //  "  Vector 1:" << vr1 << std::endl;
  }
#endif 

  // 3. Use the narrower vector (second eigenvalue) for choosing the
  // grasp points. Find the two surface points which are on the vector
  // that goes through the grasp centroid.

  // a) Use as grasp centroid the centroid of the 2D cloud projected
  // along the plane normal to the 3D point cloud. Distance of
  // projection is taken as the distance from the 3D point cloud
  // centroid to the 2D cloud centroid.
  pcl::compute3DCentroid (*grasp.cloud2D, grasp.centroid2D);
  Eigen::Vector4f centroid3D;
  pcl::compute3DCentroid (*grasp.cloud_all, centroid3D);

  Eigen::Vector4f centroid4f = 
    grasp.centroid2D + (grasp.centroid2D - centroid3D).norm() * plane;
  Eigen::Vector3f centroid(centroid4f(0), centroid4f(1), centroid4f(2));

  // b) Find surface points B for eigenvector going through centroid X
  // c) Use middlepoint of found surface points as grasp centroid Y

  // Project the centroid along the second PCA eigenvector to max
  // grasp width distance to get the points 'test1' and 'test2'
  Eigen::Vector3f test1 = centroid + vector * grasp_max_width;
  Eigen::Vector3f test2 = centroid - vector * grasp_max_width;

  // Find object surface points which are closest to 'test1' and 'test2'
  Eigen::Vector3f p1, p2;
  pca_grasping::closestPoint(cloud, kdtree, test1, p1);
  pca_grasping::closestPoint(cloud, kdtree, test2, p2);

  grasp.centroid = centroid;
  // Use distance between 'p1' and 'p2' as grasp width.
  grasp.open = (p1 - p2).norm() + grasp_safety_offset;

  // Set finger tip positions
  grasp.f1 = p1 + vector * grasp_safety_offset / 2;
  grasp.f2 = p2 - vector * grasp_safety_offset / 2;

  // Compute hand rotation in radians [0, pi].
  // Resolve orientation of the grasp 'vector' by checking angle between a
  // vector orthogonal to the finger axis and the grasp vector.
  float angle = acosf(vector.dot(finger_axis_orthogonal));
  if (angle < (PI / 2))
    grasp.rotate = acosf((-vector).dot(finger_axis));
  else 
    grasp.rotate = acosf(vector.dot(finger_axis));

  // Store grasp vector
  grasp.vector = vector;
	

//*****************************************************
    

//*****************************************************

geometry_msgs::Pose gpPose;
gpPose.position.x=grasp.centroid(0);
gpPose.position.y=grasp.centroid(1);
gpPose.position.z=grasp.centroid(2);
gpPose.orientation.x=0;
gpPose.orientation.y=0;
gpPose.orientation.z=0;
gpPose.orientation.w=1;

        pubGraspPose.publish(gpPose); //sherly publish

}





int main (int argc, char** argv)
{
  ROS_INFO("ROS NODE pca_grasping (from pca_grasping) started");
  ros::init(argc, argv, "pca_grasping");
  ros::NodeHandle nh;
  pca_grasping * g_pca = new pca_grasping(nh);
  ros::spin();
  return (0);
}

