/**
 * Pick 3D points (shift + mouse click) from a point cloud and output the 
 * coordinates
 *
 * author Joni Pajarinen 2013
 */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;

static void 
pick_callback (const pcl::visualization::PointPickingEvent &event)
{
  float x, y, z;
  event.getPoint (x, y, z);
 std::cout <<" hey the point is ";
  std::cout << x << "," << y << "," << z << std::endl; 

//             //Publish coordinates of the point
//		std_msgs::String msgStrCorners;
//		std::stringstream ss;
//                ss << x << " "  << y  << " " << z; 
//		msgStrCorners.data = ss.str();
//		po_pub.publish(msgStrCorners);
} 

static boost::shared_ptr<pcl::visualization::PCLVisualizer> 
showCloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer 
    (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> 
    rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

static void 
usage (char *filename)
{
  std::cout << "Usage: " << filename << " pcdfile" << std::endl;
}

int
main (int argc, char** argv)
{
  pcl::PCDReader reader;
  char *pcd_file;

  if (argc < 2) 
  {
    usage(argv[0]);
    return -1;
  } 
  else
    pcd_file = argv[1];

  // Load PCD file
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
  if (reader.read (pcd_file, *cloud) < 0)
  {
    std::cerr << "Could not load PCD file: " << pcd_file << std::endl;
    return(-1);
  }

  // Show point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = showCloud(cloud);

  // Register point picking callback
  viewer->registerPointPickingCallback(pick_callback);  	

  // Loop until viewer is closed
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
