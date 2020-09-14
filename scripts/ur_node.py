#!/usr/bin/env python3

import numpy as np
import urx
import math3d as m3d
import sys
import math
import time
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation as R
import copy
import random

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf


class urx_ros:
    def __init__(self, robot_ip):
        self.vel = 0.2
        self.acc = 0.2
        self.stop_acc = 0.3
        self.v_x = 0
        self.v_y = 0
        self.v_z = 0
        self.r_x = 0
        self.r_y = 0
        self.r_z = 0
        self.stop = False

        self.detection_mode = False
        self.detection_move_rad = 0.05

        self.item_height = 0.11

        self.cam_pose_correction = (0, 0, 0)
        self.holder_to_camera = (0, 0, 0)#(0.00, -0.036 ,0)

        self.robot = urx.Robot(robot_ip, True)
        self.my_tcp = m3d.Transform()  # create a matrix for our tool tcp
        self.my_tcp.pos.z = 0.21 #camera without holder
        #self.my_tcp.orient.rotate_z(math.pi)
        #self.my_tcp.orient.rotate_x(-math.pi/2)
        self.robot.set_tcp(self.my_tcp)
        self.robot.set_payload(0.0)
        time.sleep(0.2)

        self.ros_node = rospy.init_node('ur10_node', anonymous=True)
        self.pose_publisher = rospy.Publisher('ur10_pose', PoseStamped, queue_size=10)
        self.cmd_vel_subs = rospy.Subscriber("ur_cmd_vel", Twist, self.move_robot_callback, queue_size=2)
        self.cmd_pose_subs = rospy.Subscriber("ur_cmd_pose", Pose, self.move_pose_callback, queue_size=2)
        self.cmd_pose_subs = rospy.Subscriber("ur_rotate_ee", Float64, self.angle_callback, queue_size=2)
        self.cmd_pose_subs = rospy.Subscriber("ur_rotate_ee_x", Float64, self.angle_callback_x, queue_size=2)
        self.cmd_pose_subs = rospy.Subscriber("ur_detection_mode", Bool, self.detection_mode_callback)
        self.pickup_service = rospy.Service("ur_pickup", Empty, self.pick_item)
        
        self.rate = rospy.Rate(100)

        self.robot_pose = PoseStamped()
        self.seq = 1
        self.pose = []
        self.initial_pose = []
        self.center_pose = []

        self.run_node()

    def detection_mode_callback(self, detection_mode_bool):
        self.get_pose()
        self.initial_pose = copy.copy(self.pose)
        self.center_pose = copy.copy(self.pose)
        self.center_pose.pos[0] = self.center_pose.pos[0] + self.detection_move_rad

        self.detection_mode = detection_mode_bool.data
        print("detection mode received:", detection_mode_bool)
        

    def move_robot_callback(self, Twist_msg):
        self.v_x = Twist_msg.linear.x
        self.v_y = Twist_msg.linear.y
        self.v_z = Twist_msg.linear.z
        self.r_x = Twist_msg.angular.x
        self.r_y = Twist_msg.angular.y
        self.r_z = Twist_msg.angular.z

        #print("move command received:", self.v_x, self.v_y, self.v_z, self.r_x, self.r_y, self.r_z)
        if (self.v_x==0 and self.v_y==0 and self.v_z==0 and not self.stop):
                self.stop = True

    def move_pose_callback(self, Pose_msg):

        print("Pose command received:", Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w)
        
        command_attitude = R.from_quat([Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w])
        attitude_rot_vec = command_attitude.as_rotvec()
        print(attitude_rot_vec)

        self.robot.movel((Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)
        #self.robot.movel((Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), np.max([random.random(), 0.5]) * self.acc, np.max([random.random(), 0.5]) * self.vel, wait=False)

    def angle_callback(self, target_angle_msg):

        print("angle command received:", target_angle_msg)

        #self.robot.translate_tool(self.cam_pose_correction, self.acc, self.vel, wait=False) #TODO:place better

        trans = self.robot.get_pose()  # get current transformation matrix (tool to base)
        
        if abs(target_angle_msg.data) > 1:
            target_angle_msg.data = target_angle_msg.data / abs(target_angle_msg.data)

        trans.orient.rotate_z(target_angle_msg.data)

        #self.robot.set_pose(trans, wait=False, acc=0.5, vel=0.2)  # apply the new pose

    def angle_callback_x(self, target_angle_msg):

        print("angle command received:", target_angle_msg)

        #self.robot.translate_tool(self.cam_pose_correction, self.acc, self.vel, wait=False) #TODO:place better

        trans = self.robot.get_pose()  # get current transformation matrix (tool to base)
        
        if abs(target_angle_msg.data) > 1:
            target_angle_msg.data = target_angle_msg.data / abs(target_angle_msg.data)

        trans.orient.rotate_x(target_angle_msg.data)

        self.robot.set_pose(trans, wait=False, acc=0.1, vel=0.1)  # apply the new pose

    def robot_rotate_z_calback(self, angle):
        current_joints = self.robot.getj()
        

    def run_node(self):
    
        while not rospy.is_shutdown():
            self.get_pose()
            self.pose_publisher.publish(self.robot_pose)

            if (self.v_x!=0 or self.v_y!=0 or self.v_z!=0 or self.stop):
                self.robot.speedl_tool([self.v_x, self.v_y, self.v_z, self.r_x, self.r_y, self.r_z], self.acc, 1)
                self.stop = False
            # elif self.detection_mode:
                
            #     current_phase = math.atan2(self.pose.pos[1]-self.center_pose.pos[1], self.pose.pos[0]-self.center_pose.pos[0])
            #     via_phase = copy.copy(current_phase) + 0.75
            #     to_phase = copy.copy(current_phase) + 1.5

            #     via_pose = copy.copy(self.center_pose)
            #     via_pose.pos[0] = self.center_pose.pos[0] + self.detection_move_rad * math.cos(via_phase)
            #     via_pose.pos[1] = self.center_pose.pos[1] + self.detection_move_rad * math.sin(via_phase)

            #     to_pose = copy.copy(self.center_pose)
            #     to_pose.pos[0] = self.center_pose.pos[0] + self.detection_move_rad * math.cos(to_phase)
            #     to_pose.pos[1] = self.center_pose.pos[1] + self.detection_move_rad * math.sin(to_phase)

            #     self.robot.movec(via_pose, to_pose, acc=self.acc, vel=0.2*self.vel, wait=False)


            self.rate.sleep()
        
        self.cleanup()

    def get_pose(self):
        self.pose = self.robot.get_pose()

        self.robot_pose.pose.position.x = self.pose.pos[0]
        self.robot_pose.pose.position.y = self.pose.pos[1]
        self.robot_pose.pose.position.z = self.pose.pos[2]
        
        roation = R.from_matrix(self.pose.orient.list)
        quat = roation.as_quat()
        self.robot_pose.pose.orientation.x = quat[0]
        self.robot_pose.pose.orientation.y = quat[1]
        self.robot_pose.pose.orientation.z = quat[2]
        self.robot_pose.pose.orientation.w = quat[3]

        self.robot_pose.header.seq = self.seq
        self.seq = self.seq+1

        self.robot_pose.header.frame_id = "camera_frame"
        self.robot_pose.header.stamp = rospy.Time.now()


    def pick_item(self, req):

        self.robot.translate_tool(self.holder_to_camera, self.acc, self.vel, wait=False)

        rospy.sleep(1)

        last_pose = self.robot.get_pose()
        
        pickup_pose = copy.copy(last_pose)

        pickup_pose.pos.z = self.item_height
        pickup_pose.pos.x = pickup_pose.pos.x + self.cam_pose_correction[0]
        pickup_pose.pos.y = pickup_pose.pos.y + self.cam_pose_correction[1]

        command_attitude = R.from_matrix(pickup_pose.orient.list)
        attitude_rot_vec = command_attitude.as_rotvec()
        
        self.robot.movel((pickup_pose.pos.x, pickup_pose.pos.y, pickup_pose.pos.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)
        
        rospy.sleep(6)

        self.robot.set_digital_out(0, True)
        
        rospy.sleep(0.1)

        #self.robot.movel((last_pose.pos.x, last_pose.pos.y, last_pose.pos.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

        return []

    def cleanup(self):
        self.robot.close()
        
if __name__ == '__main__':
    robot = urx_ros("192.168.1.110")
    exit()
