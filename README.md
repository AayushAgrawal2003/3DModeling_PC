# 3DModeling_PC

This a ros package to create a 3D reconstructed surface using point cloud data from the end effector of a robotic manipulator.



## TODO: 

- [X] Global point cloud: World Frame  
- [X] Point cloud subscriber 
- [X] Transform listner
- [X] Transform point cloud ->  world frame 
- [X] concatenate point cloud with old point cloud 
- [X] publish point cloud 
- [X] rospy spin# 3DModeling_PC
- [ ] Convert transformation matrix to pose data
	- [ ] Log data of relative transform and global transform
	- [ ] COnvert this data to pose and then compare the values.
- [ ] Calculate rough position of the model and create an automated process for picking frames
- [ ] Fuse the 2 data sources together
- [X] Create a distance calculator
- [X] Create the filter 
- [X] Calculate the min and max value
- [ ] Make the values rosparams
