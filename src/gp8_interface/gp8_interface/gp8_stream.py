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


class SimpleQueueClient(Node):

    def __init__(self):
        super().__init__('simple_queue_client')

        profile = QoSProfile(depth=1)
        profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.call_start_traj_mode_service()

        self.queue_client = self.create_client(
            QueueTrajPoint, "/queue_traj_point")
        while not self.queue_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.reqestQ = QueueTrajPoint.Request()

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
    time_stamps = np.linspace(0,100,1000)
    for ti in time_stamps:
        queue_client.send_goal(ti, 0.1 * np.sin(ti))
        time.sleep(0.1)

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