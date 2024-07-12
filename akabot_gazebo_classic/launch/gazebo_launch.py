# Launches akabot in Gazebo Classic
#
# File adapted from https://automaticaddison.com

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="akabot_gazebo_classic").find(
        "akabot_gazebo_classic"
    )
    pkg_description = FindPackageShare(package="akabot_description").find(
        "akabot_description"
    )
    urdf_model_path = os.path.join(pkg_description, "urdf/akabot_gz_classic.urdf.xacro")
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    robot_description_config = Command(
        [
            "xacro ",
            urdf_model_path,
        ]
    )

    world_filename = "pick_and_place.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    # Launch configuration variables specific to simulation
    world = LaunchConfiguration("world")

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model to load",
    )

    robot_description_str = ParameterValue(robot_description_config, value_type=str)
    # Start robot state publisher node
    params = {"robot_description": robot_description_str, "use_sim_time": True}
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[params],
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={
            "world": world,
        }.items(),
    )

    # Spawn robot in Gazebo
    start_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "akabot",
            "-x",
            "-0.155216",
            "-y",
            "-0.056971",
            "-z",
            "1.010770",
            "-Y",
            "0.016798",
        ],
    )

    # Spawn akabot_arm_controller
    start_akabot_arm_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "akabot_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Spawn hand_controller
    start_hand_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "hand_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    # Spawn joint_state_broadcaser
    start_joint_state_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_akabot_arm_controller_cmd)
    ld.add_action(start_hand_controller_cmd)
    ld.add_action(start_joint_state_broadcaster_cmd)

    return ld
