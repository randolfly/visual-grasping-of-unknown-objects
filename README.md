# visual-grasping-of-unknown-objects

link to paper: https://aaltodoc.aalto.fi/bitstream/handle/123456789/23162/master_John_Bensam_Christina_2016.pdf?sequence=2&isAllowed=y


The novelty of the project is that the study has led to questioning the general approach used by researchers to solve the grasping problem.

## SHAF(Symmetry height accumulated features) ##
roslaunch openni_launch openni.launch

rosrun trigger trigger2.py
rosrun point_cloud_edit pc_merge
rosrun calc_grasppoints_svm calc_grasppoints

roslaunch jaco_moveit_config youbot_kinect.launch
rosrun mrs_jaco_move mrs_jaco_grasp

rostopic pub /SS/doSingleShot std_msgs/String "asdf" -r 0.2

NOTE:: 
=>the include in CHaarFeature.cpp file
"#include </home/christina/CHaarFeature.h>  " has to be changed to the include dir in the package where CHaarFeature.h is present

=>the include in calc_grasppoints.cpp file
"#include </home/christina/CIntImage_to_Featurevec.h>  " has to be changed to the include dir in the package where CIntImage_to_Featurevec.h is present

=> in the point_cloud_edit package , in the file pc_merge_with_basket_rot.hpp, the values can be changed in the following code snippet for making sure that the table is seen 
:::>void filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged,
			      bool pub_basket_position=false, float x_min=-0.95, float x_max=0.90,
			       float y_min=-0.95, float y_max=0.05, float z_min=0.11, float z_max = 0.95);// in terms of root coordinate system 


## Grasping rectangle ##

 roslaunch openni_launch openni.launch
 
rosrun point_cloud_utils util
rosrun rqt_reconfigure rqt_reconfigure     // for launching the dynamic reconfigure   

Note: after running the above commands press write_next box for saving the images. the images are saved in /tmp folder of the system. Go there and first 
=> mv scene0001c.pgm background.pgm  // save the background image 
=> mkdir rank  // create a rank folder

Now place the required object on the scene and run 

rosrun point_cloud_utils util
rosrun rqt_reconfigure rqt_reconfigure 

now check out the rank folder , it contains the images with the grasped rectangles

rosrun grasp_rect grasp_rect
roslaunch jaco_moveit_config youbot_kinect.launch
rosrun mrs_jaco_move mrs_jaco_grasp


##  PCA  ##

roslaunch openni_launch openni.launch

rosrun trigger trigger2.py
rosrun point_cloud_edit pc_merge
rosrun pca_grasping pca_grasping

roslaunch jaco_moveit_config youbot_kinect.launch
rosrun mrs_jaco_move mrs_jaco_grasp

rostopic pub /SS/doSingleShot std_msgs/String "asdf" -r 0.2

## Calibration ##

catkin make all these 5 packages
jaco_description, jaco_driver_cpp, jaco_moveit_config, jaco_cpp_lib, jaco_kinect_calib_pkg

=> roslaunch openni_launch openni.launch

Step 1:imag
roslaunch  jaco_moveit_config kinect_jaco_arm_calibration.launch
Step 2:
rosrun jaco_kinect_calib_pkg jaco_kinect_calib


## TEST CALIBRATION ##

=> roslaunch openni_launch openni.launch
=>rosrun kinect_grab_frame kinect_grab_frame test1.pcd
=>rosrun pick_points pick_points test1.pcd

=> roslaunch jaco_moveit_config youbot_kinect.launch

=>  rosrun mrs_jaco_move mrs_jaco_move

=>rosrun check_calib check_calib



