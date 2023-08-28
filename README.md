This respository specifies the process of configuring the Moveit path planner for the YASKAWA robot GP-8 (or others in the same family)
in the context of trajectory generation. 

The colcon_starter_ws includes all the files needed to move the GP-8 robot using MotoROS2.

---

ROS 2 packages in colcon_starter_ws are as follows:

cpp_pubsub --simple C++ programs to move robot without path planener
dev_opencv_py --simple Python programs to subscribe and publish camera imgages - used for camera tracking -  /queue_traj_point - control mode - it refers to gp8_stream_vision.py  in gp8_interface package
gp8_interface --simple Python programs to move robot without path planener to test different motion mode
hello_moveit -- simple C++ programs calling path-planener by Moveit
motoman_resources -- used for color and material definition of the robot only as reference
motoXY -- configuration files for RViz created by Moveit Assistant 
motoros2_client_interface_dependencies --MotoROS2 specific 
ros-industrial --MotoROS2 specific
yaskawa-global --MotoROS2 specific
motoros2_config.yaml --config file to be deployed into YRC-1000

---

#Note_1.
For simplicty the autor of the colcon_picknik_ws changed the joint names as follows
from:
[joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t]
to:
[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]

---

#Note_2.
The process of moving robot is as follows (can be simplified as one call but the author applied this way to keep simplicty during development process):
a. ros2 launch moveit_resources_moto_moveit_config  yaskawa_hw.launch.py
or
a. docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888

b. enable Yaskawa drives:
ros2 run cpp_pubsub enable_client

c. start RViz
ros2 launch moveit_resources_moto_moveit_config  xy_start.launch.py 
or
ros2 run hello_moveit joint_hello_moveit
or
ros2 run gp8_interface move_on_traj

d. disable Yaskawa drives:
ros2 run cpp_pubsub disable_client

---

#Note_3.
in motoros2_config.yaml to run robot from ROS 2 we have to mostly change (changes reflect current configuration in colcon_picknik_ws):

agent_ip_address: 192.168.255.10
agent_port_number: 8888
node_name: motoman_ros2
node_namespace: "yaskawa"
joint_names:
  - [joint_1, joint_2, joint_3,
     joint_4, joint_5, joint_6]
joint_states: default

---

Configuration & launch & run

1. Follow https://github.com/yaskawa-global/motoros2_interfaces  to configure MotoROS2 gp8_interface

2. Download from https://github.com/gavanderhoorn/motoman/tree/kinetic-devel/motoman_gp8_support
for xacro file apply (in one folder both file gp8.xacro & gp8_macro.xacro):
xacro gp8.xacro > moto.urdf

3. moto.urdf copy to moto_description pkg

4. ros2 launch moveit_setup_assistant setup_assistant.launch.py -- to setup Moveit --> output is "moto_moveit_config" pkg

5. Prepare launch files inside 
a. start.launch.py --simulation in RViz
b. xy_start.launch.py --move robot from RViz

---
