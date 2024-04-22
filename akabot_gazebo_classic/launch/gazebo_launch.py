# Launches akabot in Gazebo and can be controlled using a joystick.
#
# File adapted from https://automaticaddison.com

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    # pkg_path = FindPackageShare(package="akabot_gazebo").find("akabot_gazebo")
    pkg_description = FindPackageShare(package="akabot_description").find(
        "akabot_description"
    )
    # pkg_teleop = FindPackageShare(package="akabot_teleop").find("akabot_teleop")
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")

    # gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    # twist_mux_params_file = os.path.join(pkg_teleop, "config/twist_mux.yaml")
    world_filename = "simple.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model to load",
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={
            "world": world,
            # "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
    )

    # Spawn robot in Gazebo
    start_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "akabot"],
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
            "akabot_arm_controller",
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

    # Start joystick node
    # start_joystick_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(pkg_teleop, "launch", "joystick_launch.py")]
    #     )
    # )

    # Start twist mux
    # start_twist_mux_cmd = Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     parameters=[twist_mux_params_file, {"use_sim_time": True}],
    #     remappings=[("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")],
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_akabot_arm_controller_cmd)
    ld.add_action(start_hand_controller_cmd)
    ld.add_action(start_joint_state_broadcaster_cmd)
    # ld.add_action(start_joystick_cmd)
    # ld.add_action(start_twist_mux_cmd)

    return ld
