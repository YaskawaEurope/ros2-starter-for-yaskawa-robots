from launch import LaunchDescription
from launch_ros.actions import Node
import os, subprocess


def generate_launch_description():

    # Launch micro_ros_agent to communicate to B2
    micro_ros_agent_motoman_b2 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        output="screen",
    )

    return LaunchDescription(
        [
            micro_ros_agent_motoman_b2,
        ]
    )
