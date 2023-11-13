#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

ITR_DENSITY = 10


class Depth():

    def __init__(self):
        self.body_max = -1
        self.body_min = 2000
        self.rate = rospy.Rate(1)
        self.depth_array = []
    def pitcure_iter(self):
        while not rospy.is_shutdown():
            self.pixel2depth()
            image = cv2.imread("/home/inspire_01/catkin_ws/src/surface_pc/data/test_images/main.png")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # rospy.loginfo(str(self.body_max) + " " +str(self.body_min))
            # cv2.imshow("test" , gray)
            # cv2.waitKey(0)
            self.temp_max = -1
            self.temp_min = 2000
            dims = gray.shape   
            if len(self.depth_array) > 0:
                for i in range(0,dims[0],ITR_DENSITY):
                    for j in range(0,dims[1],ITR_DENSITY):
                        if gray[i][j] == 253:
                            if self.depth_array[i][j] >  self.temp_max: 
                                self.temp_max = self.depth_array[i][j]
                            if self.depth_array[i][j] <  self.temp_min: 
                                self.temp_min = self.depth_array[i][j] 
                
                self.body_max = self.temp_max
                self.body_min = self.temp_min

                rospy.set_param('/max_body', float(self.body_max/1000) + 0.1)
                rospy.set_param('/min_body', float(self.body_min/1000) - 0.1)
            else:
                #rospy.loginfo("No Depth Data")
                pass
            self.rate.sleep()

    def convert_depth_image(self,ros_image):
        bridge = CvBridge()
        # Use cv_bridge() to convert the ROS image to OpenCV 
        #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_image = bridge.imgmsg_to_cv2(ros_image, "passthrough")
        self.depth_array = np.array(depth_image, dtype=np.float32)
        # print ('center depth:', depth_array[x,y])
        # print(depth_array)

    def pixel2depth(self):
        rospy.Subscriber("/depth", Image,callback=self.convert_depth_image, queue_size=1)

if __name__ == '__main__':
    rospy.init_node("AdaptiveDepth")
    new = Depth()
    new.pitcure_iter()
