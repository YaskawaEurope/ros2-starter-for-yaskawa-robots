#!/usr/bin/env python

import rclpy
import math
import time
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
from motoros2_interfaces.srv import StartTrajMode


class SimpleTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('simple_trajectory_action_client')
      
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        profile = QoSProfile(depth=1)
        profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.action_client = ActionClient(
            self, FollowJointTrajectory, 'yaskawa/follow_joint_trajectory')

        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.listener_callback, profile)

        self.call_start_traj_mode_service()

    def listener_callback(self, msg: JointState):

        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort

    def call_start_traj_mode_service(self):
        client = self.create_client(StartTrajMode, "yaskawa/start_traj_mode")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting fot service")

        request = StartTrajMode.Request()

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_start_traj_mode))

    def callback_set_start_traj_mode(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(" service call failed; %r" % (e,))

    def return_joint_state(self):
        
        q = []
        
        for joint in self.name:
            index = self.name.index(joint)
            position = self.position[index]
            velocity = self.velocity[index]
            effort = self.effort[index]
            q.append(position)
            print("axis:", index + 1, "position [deg]", position)

        return q

    def send_goal(self):

        # self.return_joint_state()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()

        goal.goal_time_tolerance = Duration(seconds=0.0).to_msg()

        goal.trajectory.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q1 = list(q0)
        q2 = list(q0)
        q3 = list(q0)

        q1[5] -= math.radians(20)
        q1[0] -= math.radians(10)

        q2[0] += math.radians(20)
        q2[1] += math.radians(10)
        q2[2] += math.radians(10)

        q3[0] -= math.radians(10)
        q3[1] -= math.radians(10)
        q3[2] -= math.radians(10)

        # Make the robot come to a complete stop at each trajectory point (ie:
        # zero target velocity).
        qdot = [0.0] * len(goal.trajectory.joint_names)

        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
       # print("-----------------")
        #print(self.get_clock().now().to_msg())

        # add points
        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=q0,
                velocities=qdot,
                accelerations=qdot, effort=qdot,
                time_from_start=Duration(seconds=0.0).to_msg()
            )
        )

        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=q1,
                velocities=qdot,
                accelerations=qdot, effort=qdot,
                time_from_start=Duration(seconds=5.0).to_msg()
            )
        )

        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=q2,
                velocities=qdot,
                accelerations=qdot, effort=qdot,
                time_from_start=Duration(seconds=10.0).to_msg()
            )
        )

        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=q3,
                velocities=qdot,
                accelerations=qdot, effort=qdot,
                time_from_start=Duration(seconds=15.0).to_msg()
            )
        )

        self.get_logger().info('Waiting for driver\'s action server to become available ..')
        self.action_client.wait_for_server()
        self.get_logger().info('Connected to trajectory action server')
        self.send_goal_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info('Response callback...')
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Results callback...')
        self.return_joint_state()
        result = future.result().result
        self.get_logger().info('Result: '+str(result))

    def feedback_callback(self, feedback_msg):
       # self.get_logger().info('moving robot!')
        feedback = feedback_msg.feedback


def main(args=None):
    rclpy.init(args=args)
    action_client = SimpleTrajectoryActionClient()
    action_client.return_joint_state()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
