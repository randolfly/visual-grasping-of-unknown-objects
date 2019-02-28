#include "image_converter.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Constructor
ImageConverter::ImageConverter() : it_(nh_)
{
  image_pub_ = it_.advertise("camera/processed", 1);
  image_sub_ = it_.subscribe("/camera/ir/image_rect_ir", 1, &ImageConverter::imageCb, this);
}


// Destructor
ImageConverter::~ImageConverter() {}


// Callback function, converts the images to OpenCV-images
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr imageOut_; 

  // Try to convert the captured image to OpenCV-format
  try
  {
    imageOut_ = cv_bridge::toCvCopy(msg, enc::MONO16); // Grayscale image
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert the captured image to 8bit (OpenCV can't display 16bit?)
  (imageOut_->image).convertTo(imageOut_->image,CV_8U);

  // Save to class member
  imageIR = (imageOut_->image).clone();
}


// Captures one frame
cv::Mat ImageConverter::getOneFrame(ros::Rate r)
{
  cv::Mat frame;
 
  // First, destroy the previous image (if there is one) 
  if (!imageIR.empty())
    imageIR.release();

  // Try to capture a new image
  while (imageIR.empty())
  {
    // No data was published, try again
    ros::spinOnce();
    r.sleep();
  }
  
  // An image was captured, now return it
  frame = imageIR;

  return frame;
}

