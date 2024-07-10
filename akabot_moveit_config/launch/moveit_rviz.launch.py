import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Launch configuration
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time", default_value="False", description=""
    )

    # Set path to URDF model
    pkg_description = FindPackageShare(package="akabot_description").find(
        "akabot_description"
    )
    urdf_model_path = os.path.join(pkg_description, "urdf/akabot.urdf.xacro")

    # Moveit configuration
    moveit_config = (
        MoveItConfigsBuilder("akabot")
        .robot_description(
            file_path=urdf_model_path,
        )
        .robot_description_semantic(file_path="srdf/akabot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("akabot_moveit_config"),
        "rviz",
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    # Rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            rviz_node,
            move_group_node,
        ]
    )
