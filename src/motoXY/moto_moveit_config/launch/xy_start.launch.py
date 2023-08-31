import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import xacro


def generate_launch_description():

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

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
    motoman_robot_description = {
        "robot_description": motoman_robot_description_config.toxml()
    }

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_moto")
        .robot_description(file_path="config/moto.urdf.xacro")
        .robot_description_semantic(file_path="config/moto.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # moveit_controllers = {
    #     "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
    #     "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    # }
    # trajectory_execution = {
    #     "moveit_manage_controllers": True,
    #     "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    #     "trajectory_execution.allowed_goal_duration_margin": 0.5,
    #     "trajectory_execution.allowed_start_tolerance": 0.01,
    # }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }
    # Start the actual move_group node/action server
    # robot
    run_move_group_nodeX = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
            # {"use_sim_time": True},
        ],
        # remappings=[('/moto_controller/joint_states', '/yaskawa/joint_states')]
        # remappings=[('joint_states', '/moto_controller/joint_states'),
        #             ('joint_states', '/yaskawa/joint_states')]
    )


    # Start the actual move_group node/action server
    # simulation
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
            # {"use_sim_time": True},
        ],
        # remappings=[('/moto_controller/joint_states', '/yaskawa/joint_states')]
        remappings=[('joint_states', '/moto_controller/joint_states'),
                    ('joint_states', '/yaskawa/joint_states')]
    )


    # RViz
    rviz_base = os.path.join(
        get_package_share_directory(
            "moveit_resources_moto_moveit_config"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # {"use_sim_time": True},
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_moto_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    moto_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "yaskawa",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )
    #################################

    joint_state_publisherX = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {
                "robot_description": motoman_robot_description,
                "rate": 43,
                "source_list": ['/yaskawa/joint_states'],
            }
        ],
    )

    robot_state_publisherX = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            motoman_robot_description,
            {
                "robot_description": motoman_robot_description,
                "publish_frequency": 43.0,
                "frame_prefix": "",
            },
        ]
    )

    return LaunchDescription(
        [
            # db_arg,
            # static_tf,
            # mongodb_server_node,
            run_move_group_nodeX,
            joint_state_publisherX,
            robot_state_publisherX,
            rviz_node,
            run_move_group_node, ### _1
            #ros2_control_node,
            #joint_state_broadcaster_spawner,
            #moto_controller_spawner,
        ]
        # + load_controllers
    )
