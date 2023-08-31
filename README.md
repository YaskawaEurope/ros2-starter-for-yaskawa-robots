
## Introduction

This repository specifies the process of configuring the Moveit path planner for the YASKAWA robot GP-8 (or others in the same family) in the context of trajectory generation. 

The repository offers files (Python, C++) to move robots based on the trajectory specified inside the program.

The colcon_starter_ws includes all the files needed to move the GP-8 robot using MotoROS2.

The repository can be considered as a template for the other robots. In that case the changes for the robot configuration and name have to be applied.

The repository first explains how to configure the robot for the simulation. The other part includes information on how to run a robot using the MotoROS2 interface.

The repository includes files that were verified for ROS2 [Humble](https://docs.ros.org/en/humble/index.html).

The ROS 2 interface (MotoROS2) to the Yaskawa robot controller can be found [here](https://github.com/Yaskawa-Global/motoros2).
The instructions have to be followed accordingly in order to configure the Yaskawa Robot Controller and agent (micro-ros) Docker container to facilitate traffic to Robot controller. 

As mentioned, the repository and the following information have been produced to run GP-8 robot, however, all the steps are the same for other Yaskawa robots.

The complete process of preparing configuration files and motion applications is depicted  below,

![image](https://github.com/yeu-buchholz/ros2-starter-for-yaskawa-robots/assets/126800101/26a82a97-be87-45ee-8bb0-4a0c87b02e7d)



The MotoPlus application (MotoROS2 interface) can be downloaded from:
* [motoROS2 interface - MotoPlus app](https://github.com/yaskawa-global/motoros2/releases)
* [robot configuration](https://github.com/ros-industrial/motoman) or [other link](https://github.com/gavanderhoorn/motoman/tree/ros2)

Consider [Quick Start paragraph](#quick-start-robot-motion) to move robot at once.

## Information


The colcon_starter_ws is a ROS2 workspace and includes packages as follows:

* ```cpp_pubsub``` --simple C++ programs to move robot without path planener
* ```dev_opencv_py``` --simple Python programs to subscribe and publish camera images - used for camera tracking -  /queue_traj_point - control mode - it refers to gp8_stream_vision.py  in gp8_interface package
* ```gp8_interface``` --simple Python programs to move robot without path planner to test different motion mode
* ```hello_moveit``` -- simple C++ programs calling path-planener by Moveit
* ```motoman_resources``` -- used for color and material definition of the robot only as a reference
* ```motoXY``` -- configuration files for RViz created by Moveit Assistant 
* ```motoros2_client_interface_dependencies``` --MotoROS2 specific 
* ```ros-industrial``` --MotoROS2 specific
* ```yaskawa-global``` --MotoROS2 specific
* ```motoros2_config.yaml``` --config file to be deployed into YRC-1000

## Notes

### Note 1
For simplicity, the original joint names have been changed as follows:
From:
```bash
[joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t]
```
To:
```bash
[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
```

### Note 2
In motoros2_config.yaml to run robot from ROS 2 we have to change (changes reflect current configuration in colcon_picknik_ws) mostly:
```bash
agent_ip_address: 192.168.255.10
agent_port_number: 8888
node_name: motoman_ros2
node_namespace: "yaskawa"
joint_names:
  - [joint_1, joint_2, joint_3,
     joint_4, joint_5, joint_6]
joint_states: default
```
### Note 3

Check if in each terminal you call ROS, you source ROS 2 (e.g for running ```colcon build```)
```bash
source /opt/ros/humble/setup.bash
```

After you run ```colcon build``` you have to source
```bash
source install/setup.bash
```

## Quick-Start robot motion

The process of moving the robot is as follows (it can be simplified as one call, but for the debugging process, we keep these steps separate):

```bash
git clone 
cd colcon_starter_ws
colcon build
source /install/setup.bash

```
#### Launch micro-ros agent

```bash
ros2 launch moveit_resources_moto_moveit_config  yaskawa_hw.launch.py
```
or
```bash
sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888

```

#### Enable Yaskawa drives:

```bash
ros2 run cpp_pubsub enable_client
```

start RViz
```bash
ros2 launch moveit_resources_moto_moveit_config  xy_start.launch.py 
```
or
```bash
ros2 run hello_moveit joint_hello_moveit
```
or
```bash
ros2 run gp8_interface move_on_traj
```
#### Disable Yaskawa drives:
```bash
ros2 run cpp_pubsub disable_client
```


## Robot configuration

Follow https://github.com/yaskawa-global/motoros2_interfaces  to configure MotoROS2 gp8_interface

Now we will configure the GP-8 robot using Moveit Setup Assistant,

First, we convert ```.xacro ``` to ```.urdf``` for the robot specific.

Run following commands, folder ```test_ws``` is only temporary folder,

```bash
mkdir -p test_ws/src
cd test_ws/src
git clone https://github.com/gavanderhoorn/motoman/tree/kinetic-devel/motoman_gp8_support
git checkout ros2
cd ../..
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
cd test_ws/src/motoman/motoman_gp8_support/urdf
xacro gp8.xacro > moto.urdf

```

In current folder run,
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py 
```

For a completely new robot configuration, choose Create New Moveit and choose moto.urdf, as on the image below,

![image](https://github.com/yeu-buchholz/ros2-starter-for-yaskawa-robots/assets/126800101/d35aa9b4-bb95-4f2f-a77a-30c91317aefa)

Follow the steps described [here](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html),

Assistant creates pkg with ```.launch``` files. We do not use them in the current process.
We are going to use ```.launch``` files to run only simulation (in rViz) or run robot (in rViz) and command instruction to real robot GP-8.

The ```colcon_starter_ws```includes all necessary files. 

## Run robot simulation

run following commands,

``` bash
cd colcon_starter_ws
colcon build
source install/setup.bash
run2 ros2 launch moveit_resources_moto_moveit_config sim_start.launch.py
```

from other terminal move robots,

``` bash
cd colcon_starter_ws
source install/setup.bash
ros2 run hello_moveit sim_hello_moveit
```

## Move robot from ROS 2

There are two options to run the robot. Option 1, we build trajectory in Python or C++. Option 2, the robot path can be generated by path-planner, here we use [Moveit](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html).

In options 1 and 2, start to micro-ros (communication with Yaskawa Robot Controller)

```bash
sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888

```

#### For option 1
In other terminal, you need to enable drives, 
```bash
ros2 run cpp_pubsub enable_client
```

Confirm drive enable (```green light```)

Run in other terminal C++ programs,
```bash
ros2 run cpp_pubsub client_traj
```

or Python program,
```bash
ros2 run gp8_interface move_on_traj
```

disable drives, 
```bash
ros2 run cpp_pubsub disable_client
```

#### For option 2
On terminal run RViz,
``` bash
run2 ros2 launch moveit_resources_moto_moveit_config xy_start.launch.py
```
In other terminal, you need to enable drives, 
```bash
ros2 run cpp_pubsub enable_client
```

Confirm drive enable (```green light```)

Run C++ program (Moveit calculates the robot path, which is sent to the Robot Controller).

```bash
ros2 run hello_moveit hello_moveit
```
or
```bash
ros2 run hello_moveit joint_hello_moveit
```
or
```bash
ros2 run hello_moveit hello_obstacle
```



