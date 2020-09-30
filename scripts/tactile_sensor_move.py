#!/usr/bin/env python3

import numpy as np
import math3d as m3d
import sys
import math
import time
import rospy
from geometry_msgs.msg import Pose, Twist, Vector3, PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
import copy
import random
import tensorflow.compat.v1 as tf
import tensorflow as tf2
import matplotlib.pyplot as plt
import cv2

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf


class TactileSensor:
    def __init__(self):

        self.stop = False

        self.ros_node = rospy.init_node('tactile_sensor_image', anonymous=True)

        self.cmd_vel_pubs = rospy.Publisher("ur_cmd_vel", Twist, queue_size=2)
        self.adjusted_pose_pubs = rospy.Publisher("tactile_pose", Pose, queue_size=2)

        self.mode_subs = rospy.Subscriber("/tactile_control_mode", Bool, self.tactile_mode_callback, queue_size=2)
        self.ur_pose_subs = rospy.Subscriber("/ur10_pose", PoseStamped, self.pose_callback, queue_size=2)
        self.images_subs = rospy.Subscriber("/dvs/image_raw", Image, self.image_callback, queue_size=1)
        self.rate = rospy.Rate(10)

        self.contact_status = Bool()
        self.contact_angle = Vector3()

        self.angle_values = [0.15, 0.25]
        self.N_angles = 9
        self.list_of_rotations = []

        for i in range(1, self.N_angles):
            theta = i * 2 * math.pi/(self.N_angles - 1)
            for phi in self.angle_values:
                rx = phi * math.cos(theta)
                ry = phi * math.sin(theta)
                rotvec = [rx, ry, 0]
                self.list_of_rotations.append(rotvec)
                
        self.base_to_default_rot = R.from_quat([0, 1, 0, 0])

        self.input_frame = np.zeros(shape=(1, 260, 346, 3))

        self.bridge = CvBridge()

        #load tensorflow model
        self.sess = tf.Session()
        tf.disable_eager_execution()
        saver = tf.train.import_meta_graph('./frame_model/model.meta')
        g = tf.train.latest_checkpoint('./frame_model/')
        saver.restore(self.sess, tf.train.latest_checkpoint('./frame_model/')) 

        self.input_image = tf.get_default_graph().get_tensor_by_name("input_image:0")
        self.nn_last_layer = tf.get_default_graph().get_tensor_by_name("nn_last_layer:0")

        self.vec_of_precitions = []

        self.last_received_pose = Pose()

        self.move_robot = False

        self.converge_counter = 0

        rospy.spin()
        #plt.plot(self.vec_of_precitions)
        #plt.show()

    def pose_callback(self, pose_msg):
        self.last_received_pose = pose_msg        

    def tactile_mode_callback(self, bool_msg):
        self.move_robot = bool_msg.data
       
    def image_callback(self, image_msg): 

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        if np.shape(cv_image) == (260, 346, 3):
            self.run_prediction(cv_image)    
        

    def cropFrames(self, image, circle_center=(173, 130), circle_rad=100, im_height=260, im_width=346, im_channels=1, expand_dims=False):
        
        mask = np.zeros((im_height, im_width, im_channels), dtype=np.float32)            
        cv2.circle(mask, circle_center, circle_rad, [1]*im_channels, -1, 8, 0)
        cropped_image = np.multiply(mask, image)

        return cropped_image
        
    def run_prediction(self, input_image):

        cropped_image = self.cropFrames(input_image, circle_center=(170, 125), circle_rad=115, im_channels=3)
        normalized_image = (cropped_image - np.average(cropped_image)) / np.max(cropped_image)

        # print(np.shape(cropped_image))
        # plt.imshow(cropped_image)
        # plt.show()

        last_layer = self.sess.run([self.nn_last_layer], 
                            feed_dict={self.input_image: [normalized_image]})

            
        NN_prediction = np.argmax(last_layer) 

        vel_cmd = Twist()

        if self.move_robot:        
            if NN_prediction == 0:
                vel_cmd.linear.x = 0
                vel_cmd.linear.y = 0   
                vel_cmd.linear.z = 0.01       
                vel_cmd.angular.x = 0
                vel_cmd.angular.y = 0   
                vel_cmd.angular.z = 0 
            elif NN_prediction == 1:    
                vel_cmd.linear.x = 0
                vel_cmd.linear.y = 0   
                vel_cmd.linear.z = 0     
                vel_cmd.angular.x = 0
                vel_cmd.angular.y = 0   
                vel_cmd.angular.z = 0   
                self.converge_counter = self.converge_counter + 1
            else:
                print(NN_prediction)
                print(len(self.list_of_rotations))
                hit_angle = self.list_of_rotations[NN_prediction-2]
                norm_hit_angle = [float(i)/np.linalg.norm(hit_angle) for i in hit_angle]
                angle_rate = 0.1
                vel_cmd.linear.x = 0
                vel_cmd.linear.y = 0   
                vel_cmd.linear.z = 0     
                vel_cmd.angular.x = - angle_rate * norm_hit_angle[0]
                vel_cmd.angular.y = - angle_rate * norm_hit_angle[1]
                vel_cmd.angular.z = 0   

            
            self.cmd_vel_pubs.publish(vel_cmd)

            if self.converge_counter > 3:
                self.move_robot = False
                self.adjusted_pose_pubs(self.last_received_pose)
                self.converge_counter = 0

        print("angle_prediction")
        print(NN_prediction)
        #self.vec_of_precitions.append(NN_prediction)        


        
if __name__ == '__main__':
    robot = TactileSensor()
    exit()
