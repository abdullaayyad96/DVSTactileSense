#!/usr/bin/env python3

import numpy as np
import math3d as m3d
import sys
import math
import time
import rospy
from geometry_msgs.msg import Pose, Vector3, PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation as R
import copy
import random

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf


class DataGenerator:
    def __init__(self):

        self.stop = False

        self.ros_node = rospy.init_node('data_generator', anonymous=True)
        self.contact_stats_pub = rospy.Publisher('contact_status', Bool, queue_size=10)
        self.contact_deg_pub = rospy.Publisher("contact_angle", Vector3, queue_size=2)
        self.cmd_pose_pub = rospy.Publisher("/ur_cmd_pose", Pose, queue_size=2)
        self.cmd_pose_subs = rospy.Subscriber("/ur10_pose", PoseStamped, self.pose_callback, queue_size=2)
        self.rate = rospy.Rate(100)

        self.robot_pose = PoseStamped()
        self.cmd_pose = Pose()
        self.cmd_pose.position.x = -0.2
        self.cmd_pose.position.y = -0.6

        self.contact_status = Bool()
        self.contact_angle = Vector3()

        self.angle_values = [0.15, 0.25]

        self.contact_thresh = -0.012#-0.005
        self.max_z = 0.1
        self.min_z = -0.0158#-0.01

        self.N_examples = 9
        self.N_iterations_per_example = 5

        self.base_to_default_rot = R.from_quat([0, 1, 0, 0])

        self.run_node()

    def pose_callback(self, pose_msg):
        self.robot_pose = pose_msg
        
        if self.robot_pose.pose.position.z < self.contact_thresh:
            self.contact_status.data = True
        else:
            self.contact_status.data = False
            
        base_to_cam_rot = R.from_quat([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])

        cam_from_default_rot = R.from_matrix( np.matmul(np.transpose(self.base_to_default_rot.as_matrix()), base_to_cam_rot.as_matrix()))
        camera_from_default_rotvec = cam_from_default_rot.as_rotvec()
        self.contact_angle.x = camera_from_default_rotvec[0]
        self.contact_angle.y = camera_from_default_rotvec[1]
        self.contact_angle.z = camera_from_default_rotvec[2]

        self.contact_deg_pub.publish(self.contact_angle)
        self.contact_stats_pub.publish(self.contact_status)


        
    def run_node(self):

        target_quat = self.base_to_default_rot.as_quat()
        self.cmd_pose.orientation.x = target_quat[0]
        self.cmd_pose.orientation.y = target_quat[1]
        self.cmd_pose.orientation.z = target_quat[2]
        self.cmd_pose.orientation.w = target_quat[3]
        self.cmd_pose.position.z = self.max_z
        self.cmd_pose_pub.publish(self.cmd_pose)
        rospy.sleep(5)
        
        for i in range(self.N_examples):
            if i==0:
                rx = 0
                ry = 0
                rz = 0

                target_rot = R.from_matrix(self.base_to_default_rot.as_matrix() * R.from_rotvec([rx, ry, rz]).as_matrix())
                target_quat = target_rot.as_quat()
                self.cmd_pose.orientation.x = target_quat[0]
                self.cmd_pose.orientation.y = target_quat[1]
                self.cmd_pose.orientation.z = target_quat[2]
                self.cmd_pose.orientation.w = target_quat[3]

                for j in range(self.N_iterations_per_example):
                    self.cmd_pose.position.z = self.max_z
                    self.cmd_pose_pub.publish(self.cmd_pose)
                    rospy.sleep(2)

                    self.cmd_pose.position.z = self.min_z
                    self.cmd_pose_pub.publish(self.cmd_pose)
                    rospy.sleep(3)

            else:
                theta = i * 2 * math.pi/(self.N_examples - 1)
                for phi in self.angle_values:
                    rx = phi * math.cos(theta)
                    ry = phi * math.sin(theta)
                    rz = 0
                    
                    target_rot = R.from_matrix(np.matmul(self.base_to_default_rot.as_matrix(), R.from_rotvec([rx, ry, rz]).as_matrix()))
                    target_quat = target_rot.as_quat()
                    self.cmd_pose.orientation.x = target_quat[0]
                    self.cmd_pose.orientation.y = target_quat[1]
                    self.cmd_pose.orientation.z = target_quat[2]
                    self.cmd_pose.orientation.w = target_quat[3]

                    for j in range(self.N_iterations_per_example):
                        self.cmd_pose.position.z = self.max_z
                        self.cmd_pose_pub.publish(self.cmd_pose)
                        rospy.sleep(2)

                        self.cmd_pose.position.z = self.min_z
                        self.cmd_pose_pub.publish(self.cmd_pose)
                        rospy.sleep(3)

        
if __name__ == '__main__':
    robot = DataGenerator()
    exit()
