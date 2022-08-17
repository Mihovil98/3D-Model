# Creating 3D models of objects using a robotic hand and a 3D camera

## Abstract

A system for creating 3D models of objects is considered. It consists of the physical components robotic arm Universal Robots UR5 and 3D camera Intel RealSense LiDAR L515, and a software solution implemented using the Python programming language that are connected within the ROS framework.

The final 3D model of the object represents a set of point clouds transformed from a camera coordinate system to a robot base coordinate system. The results of the experimental evaluation are presented by a colored 3D point cloud obtained by fusion of three RGB-D images acquired from different angles.

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

![multiple_back00](https://user-images.githubusercontent.com/74960514/185081044-46ae4125-0b58-4a66-940b-88920900cdc4.png)
![multiple_back01](https://user-images.githubusercontent.com/74960514/185081051-dba93fba-8fb3-4197-903e-a15c19572e01.png)
![multiple_back02](https://user-images.githubusercontent.com/74960514/185081054-242cacbc-1d0e-4e2a-873d-364bf5b6d5d9.png)
![multiple_front00](https://user-images.githubusercontent.com/74960514/185081067-74a32b57-ab39-4020-b6de-f63b22fe21ec.png)
![multiple_front01](https://user-images.githubusercontent.com/74960514/185081071-7055c306-6c77-47c7-80de-972b5d9edb89.png)
![multiple_front02](https://user-images.githubusercontent.com/74960514/185081074-66778efe-b1ae-4fcd-9582-207a555add3a.png)
