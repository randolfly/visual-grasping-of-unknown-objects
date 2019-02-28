#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL17 specific includes
#include <pcl/conversions.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


char *outputfile = NULL;

static bool capture = false;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (ros::isShuttingDown()) return;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);

 // pcl_conversions::toPCL(*msg, *cloud);

  if (capture)
  {
    // Save the point cloud to disk
    pcl::PCDWriter writer;
    writer.writeBinary<pcl::PointXYZRGB> (outputfile, cloud);
    
    ROS_INFO_STREAM("Saved " << cloud.points.size () << " data points to " << 
		    outputfile);

    // Shutdown
    ros::shutdown();
  }
  capture = true;
}

void usage (char *filename)
{
  ROS_WARN_STREAM("Usage: " << filename << " outputfile.pcd");
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_grab_frame");
  ros::NodeHandle nh;

  if (argc < 2) 
  {
    usage(argv[0]);
    // Shutdown
    ros::shutdown();
  } 
  else
  {
    outputfile = argv[1];
  }

  // Create a ROS subscriber for the openni Kinect point cloud
  // Note: openni must have been reconfigured to create registered point clouds
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, 
				      cloud_cb);

  // Spin
  ros::spin ();
}

/* 
   TODO: 

   Grab kinect pointclouds
   for n = 1 to N
     put object n on table, remove other objects
     grab Kinect pointcloud and save it to disk as a n.PCD file
   end

   Extract object pointclusters from pointclouds
   for n = 1 to N
     load n.PCD and extract clusters from it
     save cluster that is most likely the object n as n.cluster.PCD
     TODO: detect object according to distance and position from Kinect?
   end

   Create VFH KD-tree from PCD-files
   Load all N object pointclusters. Compute a VFH-histogram for each
   cluster and create a KD-tree for doing a nearest neighbour search. 


   OBSERVATION PROBABILITY COMPUTATION POSSIBILITIES

   1. To recognize an object in our problem setup, it could actually be a
   good idea to create a Gaussian mixture, where each Gaussian mixture
   component corresponds to several VFH-histograms of one
   object. Trying to match a new VFH-histogram to the training set,
   would then yield the probability of each mixture component, that
   is, the likelihood for each object.

   2. Take pixel clouds of multiple objects. Compute distance to each
   VFH-signature for each object. Compute probabilities for correct
   object classification by using reference setups with human
   classified objects, that is, estimate from correct labelings the
   effect of VFH-distances to the classification probability.

*/
