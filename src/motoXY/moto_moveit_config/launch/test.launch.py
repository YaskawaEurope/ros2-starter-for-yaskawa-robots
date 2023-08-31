from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import xacro

def generate_launch_description():
    # Get the path to the robot description file
    robot_description_path = get_package_share_directory('my_robot') + '/urdf/my_robot.urdf'

    motoman_directory = "moveit_resources_moto_moveit_config"

    # URDF
    motoman_robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory(motoman_directory),
            "config",
            "moto.urdf.xacro",
        ),
        # mappings={'prefix': '/yaskawa_a1/'},
    )

    # Launch Gazebo with the robot spawn script
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': get_package_share_directory('my_robot') + '/worlds/my_world.world'}.items(),
    )

    # Launch the robot spawn script
    spawn_robot_script = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/spawn_entity.py']),
        launch_arguments={'entity': 'robot', 'name': 'my_robot', 'xml': motoman_robot_description_config}.items(),
    )

    return LaunchDescription([gazebo_launch_file, spawn_robot_script])
