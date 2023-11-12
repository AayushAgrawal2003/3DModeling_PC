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
	- [ ] Make one file for both ICP and global pose 
	- [ ] Write script for transform to pose estimation 
	- [ ] If the value of the estimated difference between the ICP and global pose then drop the data from the frame. 
- [ ] Calculate rough position of the model and create an automated process for picking frames
- [X] Create a distance calculator
- [X] Create the filter 
- [X] Calculate the min and max value
- [X] Make the values rosparams
