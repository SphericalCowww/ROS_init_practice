import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
##############################################################################################################################
def generate_launch_description():
    ld = LaunchDescription() 
    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            os.path.join(
                get_package_share_directory("robot_build"),
                "urdf",
                "arduinobot.urdf.xacro",
            ),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package   ="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,    
        }],
    )

    ### bugs out, see: https://github.com/ros-controls/gz_ros2_control/issues/567
    controller_manager = Node(
        package   ="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("robot_controller"),
                "config",
                "arduinobot_controller.yaml",
            ),
        ],
    )
    arm_controller_spawner = Node(
        package   ="controller_manager",
        executable="spawner",
        arguments = [
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    gripper_controller_spawner = Node(
        package   ="controller_manager",
        executable="spawner",
        arguments = [
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package   ="controller_manager",
        executable="spawner",
        arguments = [
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(arm_controller_spawner)
    ld.add_action(gripper_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld
