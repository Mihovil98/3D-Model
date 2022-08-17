#!/usr/bin/env python

import rospy
import subprocess
import glob
import open3d
import numpy
import moveit_commander

def save_and_transform_point_cloud(name):
    # SPREMANJE POINT CLOUD-A

    save = subprocess.Popen(['rosrun', 'pcl_ros', 'pointcloud_to_pcd', 'input:=/camera/depth/color/points', '_prefix:={0}'.format(name)])
    rospy.sleep(1)
    save.kill()

    # SPREMANJE POZICIJE ALATA

    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)
    current_pose = move_group.get_current_pose().pose

    # KREIRANJE RTA MATRICE

    quaternion = [current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z]
    translation = [current_pose.position.x, current_pose.position.y, current_pose.position.z]

    q0 = quaternion[0]
    q1 = quaternion[1]
    q2 = quaternion[2]
    q3 = quaternion[3]

    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    tx = translation[0]
    ty = translation[1]
    tz = translation[2]

    RTA = numpy.array([[r00, r01, r02, tx],
                    [r10, r11, r12, ty],
                    [r20, r21, r22, tz],
                    [0, 0, 0, 1]])

    # IZDVAJANJE TOČAKA I BOJA IZ POINT CLOUD-A

    filename = glob.glob('{0}*.pcd'.format(name))[0]
    pcd = open3d.io.read_point_cloud(filename)
    CP = numpy.asarray(pcd.points)
    rgb = numpy.asarray(pcd.colors)

    # BRISANJE VIŠAKA

    while glob.glob('{0}*.pcd'.format(name)):
        filename = glob.glob('{0}*.pcd'.format(name))[0]
        subprocess.run(['rm', '-r', filename])

    # TRANSFORMACIJA TOČAKA U KOORDINATNI SUSTAV BAZE

    CP = numpy.insert(CP, 3, 1, axis=1)
    CP = numpy.transpose(CP)

    ATC = [[0, 1, 0, -0.11], [-1, 0, 0, 0], [0, 0, 1, -0.04], [0, 0, 0, 1]]

    AP = numpy.dot(ATC, CP)

    RP = numpy.dot(RTA, AP)
    RP = RP[:-1]
    RP = numpy.transpose(RP)

    # KREIRANJE KONAČNOG POINT CLOUD-A U KOORDINATNOM SUSTAVU BAZE

    ply = open3d.geometry.PointCloud()
    ply.points = open3d.utility.Vector3dVector(RP)
    ply.colors = open3d.utility.Vector3dVector(rgb)
    open3d.io.write_point_cloud('RP_{0}.ply'.format(name), ply)

def create_3d_model():
    rospy.init_node('robot_control', anonymous=True)
    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = 0
    joint_goal[1] = -11*numpy.pi/36
    joint_goal[2] = 5*numpy.pi/12
    joint_goal[3] = -numpy.pi/6
    joint_goal[4] = -11*numpy.pi/36
    joint_goal[5] = numpy.pi/2

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    rospy.sleep(1)

    save_and_transform_point_cloud('first')

    joint_goal[0] = 0
    joint_goal[1] = -25*numpy.pi/36
    joint_goal[2] = -5*numpy.pi/12
    joint_goal[3] = -8*numpy.pi/9
    joint_goal[4] = 11*numpy.pi/36
    joint_goal[5] = numpy.pi/2

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    rospy.sleep(1)

    save_and_transform_point_cloud('second')

    joint_goal[0] = 4*numpy.pi/9
    joint_goal[1] = -5*numpy.pi/36
    joint_goal[2] = numpy.pi/18 
    joint_goal[3] = -numpy.pi/18
    joint_goal[4] = -19*numpy.pi/36
    joint_goal[5] = numpy.pi/2

    move_group.go(joint_goal, wait=True)
    move_group.stop()
    
    rospy.sleep(1)

    save_and_transform_point_cloud('third')

if __name__ == '__main__':
  create_3d_model()