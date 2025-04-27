import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

##############################################################################################################################
def generate_launch_description():
    ld = LaunchDescription() 

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (
        MoveItConfigsBuilder(
            "arduinobot", 
            package_name="arduinobot_controller"
        ).robot_description(os.path.join(
            get_package_share_directory("robot_description"),
            "urdf",
            "arduinobot.urdf.xacro",    
        ).robot_description_semantic(
            file_path="config/arduinobot.srdf",
        ).trajectory_execution(
            file_path="config/moveit_controllers.yaml",
        ).to_moveit_configs(), 
    )
    move_group_node = Node(
        package   ="moveit_ros_move_group",
        executable="move_group",
        output    ="screen",
        parameters=[
            moveit_config.to_dict(), 
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True}
        ],
        arguments=[
            "--ros-args", 
            "--log-level", 
            "info",
        ],
    )

    rviz_node = Node(
        package   ="rviz2",
        executable="rviz2",
        name      ="rviz2",
        output    ="screen",
        arguments =[
            "-d", 
            os.path.join(
                get_package_share_directory("robot_controller"), 
                "rviz", 
                "moveit.rviz",
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    ld.add_action(is_sim_arg)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld
