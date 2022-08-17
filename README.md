# Creating 3D models of objects using a robotic hand and a 3D camera

## Abstract

A system for creating 3D models of objects is considered. It consists of the physical components robotic arm Universal Robots UR5 and 3D camera Intel RealSense LiDAR L515, and a software solution implemented using the Python programming language that are connected within the ROS framework. The final 3D model of the object represents a set of point clouds transformed from a camera coordinate system to a robot base coordinate system. The results of the experimental evaluation are presented by a colored 3D point cloud obtained by fusion of three RGB-D images acquired from different angles.

## Description

The developed system implies that the object is positioned in a certain place in relation to the robot and that the object must be such that it fits in the field of view of the camera, otherwise it needs to be changed camera position and orientation. The process of saving and transforming a point cloud into a coordinate one the robot base system is automated.

![Eksperimentalni postav](https://user-images.githubusercontent.com/74960514/185080074-c36495d9-8963-4b9e-9c0b-a1fe3b75a72a.jpeg)

The conducted experiments showed significant deviations in the transformation of point clouds recorded from different angles into a common coordinate system. Better results would be achieved through application of appropriate 3D point cloud registration methods, such as the Iterative Closest Point.

## Modules and Libraries Used

- rospy
- subprocess
- glob
- open3d
- numpy
- moveit_commander
