#!/usr/bin/env python3cd

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
import tensorflow.compat.v1 as tf
import tensorflow as tf2
import matplotlib.pyplot as plt
import cv2

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf


class TactileSensor:
    #TODO: We assume the events are being published at a rate higher than 10 Hz
    def __init__(self):

        self.stop = False

        self.ros_node = rospy.init_node('tactile_sensor', anonymous=True)
        self.contact_stats_pub = rospy.Publisher('contact_status', Bool, queue_size=10)
        self.contact_deg_pub = rospy.Publisher("contact_angle", Vector3, queue_size=2)
        self.cmd_pose_subs = rospy.Subscriber("/dvs/events", EventArray, self.event_callback, queue_size=2)
        self.rate = rospy.Rate(10)

        self.contact_status = Bool()
        self.contact_angle = Vector3()

        self.angle_values = [0.1, 0.25]
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

        self.input_frame = np.zeros(shape=(1, 1, 260, 346, 1))

        self.prev_hidden_state = np.zeros(shape=(1, 29, 39, 20))
        self.prev_carry_state = np.zeros(shape=(1, 29, 39, 20))

        #load tensorflow model
        self.sess = tf.Session()
        tf.disable_eager_execution()
        saver = tf.train.import_meta_graph('./model.meta')
        g = tf.train.latest_checkpoint('./')
        saver.restore(self.sess, tf.train.latest_checkpoint('./')) 

        self.input_image = tf.get_default_graph().get_tensor_by_name("input_image:0")
        self.initial_hidden_state = tf.get_default_graph().get_tensor_by_name('initial_hidden_state:0')
        self.initial_carry_state = tf.get_default_graph().get_tensor_by_name('initial_carry_state:0')
        self.nn_last_layer = tf.get_default_graph().get_tensor_by_name("nn_last_layer:0")
        self.hidden_state = tf.get_default_graph().get_tensor_by_name("hidden_state:0")
        self.carry_state = tf.get_default_graph().get_tensor_by_name("carry_state:0")

        self.run_node()

    def event_callback(self, event_msg): 
        
        for event in event_msg.events:
            if event.polarity > 0:
                self.input_frame[0, 0, event.y, event.x, 0] = self.input_frame[0, 0, event.y, event.x, 0] + 1
            else:
                self.input_frame[0, 0, event.y, event.x, 0] = self.input_frame[0, 0, event.y, event.x, 0] - 1

    def cropFrames(self, image, circle_center=(173, 130), circle_rad=100, im_height=260, im_width=346, im_channels=1):
        
        mask = np.zeros((im_height, im_width, im_channels), dtype=np.float32)            
        cv2.circle(mask, circle_center, circle_rad, [1]*im_channels, -1, 8, 0)
        cropped_image = np.multiply(mask, image)

        return cropped_image



        
    def run_node(self):
        prev_state = 0
        fall_counter = 0
        initiate_counter = False
        vec = []
        while not rospy.is_shutdown():
            self.rate.sleep()
            cropped_image = self.cropFrames(self.input_frame, circle_center=(170, 125), circle_rad=115)

            last_layer, final_hidden_state, final_carry_state = self.sess.run([self.nn_last_layer, self.hidden_state, self.carry_state], 
                                feed_dict={self.input_image: cropped_image, self.initial_hidden_state: self.prev_hidden_state, self.initial_carry_state: self.prev_carry_state})

            
            self.input_frame = np.zeros(shape=(1, 1, 260, 346, 1))
            NN_prediction = np.argmax(last_layer.reshape(-1), 0) 

            if (prev_state > 0) and NN_prediction==0:
                initiate_counter = True

            if initiate_counter and NN_prediction==0:
                fall_counter = fall_counter + 1
            elif NN_prediction!=0:
                initiate_counter = False
                fall_counter = 0

            if fall_counter > 5:
                self.prev_hidden_state = np.zeros(shape=(1, 29, 39, 20), dtype=float)
                self.prev_carry_state = np.zeros(shape=(1, 29, 39, 20), dtype=float)
                initiate_counter = False
                fall_counter = 0
            else:        
                self.prev_hidden_state = final_hidden_state    
                self.prev_carry_state = final_carry_state

            prev_state = NN_prediction

            print("angle_prediction")
            print(NN_prediction)
            vec.append(NN_prediction)

        plt.plot(vec)
        plt.show()


        
if __name__ == '__main__':
    robot = TactileSensor()
    exit()
