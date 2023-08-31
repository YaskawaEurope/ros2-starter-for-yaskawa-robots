#!/usr/bin/env python

import rclpy
import math
import time
import numpy as np
import sys
from os import times
from functools import partial

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from motoros2_interfaces.srv import StartPointQueueMode
from motoros2_interfaces.srv import QueueTrajPoint


from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library


class SimpleQueueClient(Node):

    def __init__(self):
        super().__init__('simple_queue_client_with_vision')

        self.target = 10000000.0

        profile = QoSProfile(depth=1)
        profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.call_start_traj_mode_service()

        self.queue_client = self.create_client(
            QueueTrajPoint, "/queue_traj_point")
        while not self.queue_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.reqestQ = QueueTrajPoint.Request()

        ########################################
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        self.subscription
        self.br = CvBridge()
        ########################################


    def listener_callback(self, data):
        """
        Callback function for camera.
        """
        print("this is a callback for the vision processing.....................")
        THRESHOLD_LOW = (110, 50, 50)
        THRESHOLD_HIGH = (130, 255, 255)
        MIN_RADIUS = 2
        
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        imgX = self.br.imgmsg_to_cv2(data)
        img = cv2.resize(imgX, (600, 480))

        # Blur image to remove noise
        img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

        # Convert image from BGR to HSV
        img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)

        # Set pixels to white if in color range, others to black (binary bitmap)
        img_binary = cv2.inRange(img_filter.copy(), THRESHOLD_LOW, THRESHOLD_HIGH)

        # Dilate image to make white blobs larger
        img_binary = cv2.dilate(img_binary, None, iterations = 1)

        # Find center of object using contours instead of blob detection. From:
        # http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        img_contours = img_binary.copy()
        contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, \
            cv2.CHAIN_APPROX_SIMPLE)[-2]

        # Find the largest contour and use it to compute the min enclosing circle
        center = None
        radius = 0
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius < MIN_RADIUS:
                    center = None

        # Print out the location and size (radius) of the largest detected contour
        if center != None:
            # print(center[0])
            self.target = center[0]
        #print (str(center) + " " + str(radius))   

        # Draw a green circle around the largest enclosed contour
        if center != None:
            cv2.circle(img, center, int(round(radius)), (0, 255, 0))
        


        cv2.imshow('webcam', img)
        #cv2.imshow('binary', img_binary)
        #cv2.imshow('contours', img_contours)
        cv2.waitKey(1)


    def call_start_traj_mode_service(self):
        client = self.create_client(
            StartPointQueueMode, "/start_point_queue_mode")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting fot service")

        self.get_logger().warn("service OK...")
        request = StartPointQueueMode.Request()

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_start_traj_mode))

    def callback_set_start_traj_mode(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(" service call failed; %r" % (e,))

    def send_goal(self, time_i, q_i):
        print ("q_i= ", q_i)
        q_iii = 1 / self.target
        q_ii = q_iii * 15.0
        print("target= ",q_ii)
        # q_i = 1.0
        if (self.target > 250.0):
            q_i = -q_i
        joint_names = [
            'group_1/joint_1',
            'group_1/joint_2',
            'group_1/joint_3',
            'group_1/joint_4',
            'group_1/joint_5',
            'group_1/joint_6'
        ]
      
        q0 = [0.0, 0.0, 0.0, 0.0, q_i, 0.0]

        qdot = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        goalQ = JointTrajectoryPoint(
            positions=q0,
            velocities=qdot,
            accelerations=qdot, effort=qdot,
            time_from_start=Duration(seconds=time_i).to_msg()
        )
        self.reqestQ.joint_names = joint_names
        self.reqestQ.point = goalQ
        self.future = self.queue_client.call_async(self.reqestQ)
        rclpy.spin_until_future_complete(self, self.future)


def main(args=None):
    rclpy.init(args=args)
    queue_client = SimpleQueueClient()
    time_stamps = np.linspace(0,1.2,100)
    time_stamps = np.linspace(0,10.2,1000)
    for ti in time_stamps:
        queue_client.send_goal(ti, ti)
        time.sleep(0.1)
        #queue_client.send_goal(ti, 0.1 * np.sin(ti))
        #time.sleep(0.1)
    #rclpy.spin(queue_client)

if __name__ == '__main__':
    main()


"""
def main(args=None):
    rclpy.init(args=args)
    queue_client = SimpleQueueClient()
    time_stamps = np.linspace(0,100,1000)
    for ti in time_stamps:
        queue_client.send_goal(ti, 0.5 * np.sin(ti))
        time.sleep(0.1)
"""