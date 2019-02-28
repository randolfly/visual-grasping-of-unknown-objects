#!/usr/bin/python

## David Fischinger
## 22.1.2014
## first version: 27.04.2011
# 
# !!! IMPORTANT !!!  only works if triggered not more than once in 5 seconds !!!!!!!!!!
# (rostopic pub /SS/doSingleShot std_msgs/String "asdf" -r 0.2) for cyclic triggering
#
# subscribes to /SS/doSingleShot", if String comes in:
#     subscribes to /camera/depth_registered/points and /camera/rgb/image_color
# publishes 
# publishes rgb image for camera1  and point cloud for camera1
# after that it unregisters subscribers for /camera/rgb/image_color and /camera/depth_registered/points and
# subscribes to camera2 (
# subscribes to point cloud of camera2 ("/camera2/depth_registered/points")
# publishes point cloud for camera2 one time and unregisters



#!/usr/bin/python


PKG = 'trigger'
import roslib; roslib.load_manifest(PKG)
import rospy
import subprocess
import time, sys
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class Trigger():
    def __init__(self, parent=None):

        #Subscriber
        ss_sub = rospy.Subscriber("/SS/doSingleShot", String, self.start_shot, queue_size=1) #neu
        self.pc_sub = None
        #Publisher
        self.pc_pub = rospy.Publisher("/SS/camera/depth_registered/points", PointCloud2 )
         
        self.pc_ = None
        self.t = None
        self.doPub = False
        
        self.pc_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pc_callback, queue_size=1)
    
    #triggers the process for publishing 
    def start_shot(self, msg):
        print "-> start shot() (set doPub True)"
        self.doPub = True
 
    
    #starts method to publish point cloud for camera1; unregisters subscriber camera1
    def pc_callback(self,msg):
        #print "-> pc_callback()"
        if self.doPub:
            self.doPub = False
            self.pc_ = msg
            self.do_publish_cam1()
            
        

    #publishes pc for cam1 and starts subscriber to camera2
    def do_publish_cam1(self):
        print "-> do_publish_cam1"
        self.t = rospy.Time.now()
        if self.pc_ == None:
            print "!!!!!!!\n\n\n\n!!!!!!!!!!!!!!!!!!!!!!!!!! trigger2.py: do_publish_cam1(): no point cloud!!!!!!!!!!!\n\n\n\n!!!!!!!!!!!!!!!!!!!!!!!"
            return
        self.pc_.header.stamp = self.t
        self.pc_pub.publish(self.pc_)
        print "trigger2.py: do_publish_cam1(): pointcloud published"
        time.sleep(0.2)
        #self.subscribe_cam2()
        self.pc_ = None
 



   

def main(args):        
    rospy.init_node('trigger', anonymous=False)
    print "rosnode trigger was started"

    trig = Trigger()
    rospy.spin()
    
if __name__ == "__main__":        
    main(sys.argv)



   

    
