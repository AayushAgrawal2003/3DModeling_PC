#!/usr/bin/env python

import numpy as np 
from sklearn.cluster import DBSCAN
import rospy 
from tqdm import tqdm
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
import struct

rospy.init_node('filtering')

white = struct.unpack('I', struct.pack('BBBB', 255, 0, 0, 255))[0]
class Density():
    def __init__(self):
        self.point_list = np.array([[0,0,0,white]])                                                                                                                                                                                                               
        # self.sub = rospy.Subscriber("/distance_filtered", PointCloud2,self.callback,queue_size=1)  
        self.msg = rospy.wait_for_message("/distance_filtered", PointCloud2,timeout=5)
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                # PointField('rgb', 12, PointField.UINT32, 1),
                PointField('rgba', 12, PointField.UINT32, 1),
                ]
        self.world_pc = rospy.Publisher("/point_cloud_world", PointCloud2, queue_size = 2)
        self.callback(self.msg)

    def callback(self, msg):
        points = list(pc2.read_points(msg, skip_nans=True))
        
        itr = 0
        for point in tqdm(points):
            if itr == 50:
                self.point_list = np.append(self.point_list,[[point[0],point[1],point[2],white]],axis =0)
                itr =0 
            else: 
                itr +=1 

        self.point_list = self.point_list 
        print(self.point_list)

        clustering = DBSCAN(eps=0.01, min_samples=1).fit(self.point_list)
        plt.hist(clustering.labels_)
        plt.show()
        print(np.unique(clustering.labels_))

        header = Header()
        header.frame_id = "camera_color_optical_frame"
        header.stamp =  rospy.Time(0)

        print(list(self.point_list))
        self.pc2_world = point_cloud2.create_cloud(header, self.fields, )

        self.world_pc.publish(self.pc2_world)

        
        

if __name__ == "__main__":
    main = Density()


        









