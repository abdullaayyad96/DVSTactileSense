#!/usr/bin/env python3

import numpy as np
import math3d as m3d
import sys
import math
import time
import rospy
from dvs_msgs.msg import EventArray
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
        self.rate = rospy.Rate(100)

        self.robot_pose = PoseStamped()
        self.cmd_pose = Pose()
        self.cmd_pose.position.x = 0
        self.cmd_pose.position.y = -0.7
        self.cmd_pose.position.z = 0.7

        self.angle_values = [0.1, 0.25]

        self.contact_thresh = -0.005
        self.max_z = 0.1
        self.min_z = -0.01

        self.N_examples = 4
        self.N_iterations_per_example = 10

        self.base_to_default_rot = R.from_quat([0, 0.707, -0.707, 0])

        self.run_node()


        
    def run_node(self):

        target_quat = self.base_to_default_rot.as_quat()
        self.cmd_pose.orientation.x = target_quat[0]
        self.cmd_pose.orientation.y = target_quat[1]
        self.cmd_pose.orientation.z = target_quat[2]
        self.cmd_pose.orientation.w = target_quat[3]
        self.cmd_pose_pub.publish(self.cmd_pose)
        print(target_quat)
        rospy.sleep(1)
        
        for j in range(2):
            for i in range(4):
                if i==0:
                    rx = 0.3
                    ry = 0
                    rz = 0
                elif i==1:
                    rx = -0.3
                    ry = 0
                    rz = 0
                if i==2:
                    rx = 0
                    ry = -0.3
                    rz = 0
                if i==3:
                    rx = 0
                    ry = 0.3
                    rz = 0

                target_rot = R.from_matrix(np.matmul(self.base_to_default_rot.as_matrix(), R.from_rotvec([rx, ry, rz]).as_matrix()))
                target_quat = target_rot.as_quat()
                print(i)
                print(target_quat)
                self.cmd_pose.orientation.x = target_quat[0]
                self.cmd_pose.orientation.y = target_quat[1]
                self.cmd_pose.orientation.z = target_quat[2]
                self.cmd_pose.orientation.w = target_quat[3]

                self.cmd_pose_pub.publish(self.cmd_pose)
                rospy.sleep(1.5)



                rx = 0
                ry = 0
                rz = 0
                target_rot = R.from_matrix(np.matmul(self.base_to_default_rot.as_matrix(), R.from_rotvec([rx, ry, rz]).as_matrix()))
                target_quat = target_rot.as_quat()
                print(target_quat)
                self.cmd_pose.orientation.x = target_quat[0]
                self.cmd_pose.orientation.y = target_quat[1]
                self.cmd_pose.orientation.z = target_quat[2]
                self.cmd_pose.orientation.w = target_quat[3]
                self.cmd_pose_pub.publish(self.cmd_pose)
                rospy.sleep(1.5)

        
if __name__ == '__main__':
    robot = DataGenerator()
    exit()
