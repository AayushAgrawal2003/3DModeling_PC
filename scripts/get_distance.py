#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

def convert_depth_image(ros_image):
    bridge = CvBridge()
    # Use cv_bridge() to convert the ROS image to OpenCV 
    #Convert the depth image using the default passthrough encoding
    depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    depth_array = np.array(depth_image, dtype=np.float32)
    depth_image = bridge.imgmsg_to_cv2(ros_image, "passthrough")
    depth_array = np.array(depth_image, dtype=np.float32)
    print ('center depth:', depth_array[240,140])
    print(depth_array)
    # print 'center depth:', depth_array[[cy], [cx]]
    # rospy.loginfo(depth_array)

def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/depth", Image,callback=convert_depth_image, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	pixel2depth()