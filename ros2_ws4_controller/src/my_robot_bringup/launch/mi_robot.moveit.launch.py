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
    robot_description_path = get_package_share_path('my_robot_description')
    robot_bringup_path     = get_package_share_path('my_robot_bringup')
    
    urdf_path          = os.path.join(robot_description_path, 'urdf', 'mi_robot.urdf.xacro')
    rviz_config_path   = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')
    robot_controllers  = os.path.join(robot_bringup_path, 'config', 'mi_robot_controllers.yaml')
    moveit_config_path = os.path.join(robot_bringup_path, 'config', 'mi_robot_moveit.srdf')

    moveit_config = (MoveItConfigsBuilder(
        "arduinobot", 
        package_name="robot_controller"
        ).robot_description(         urdf_path
        ).robot_description_semantic(moveit_config_path
        ).trajectory_execution(      robot_controllers
        ).to_moveit_configs() 
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(), 
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=[
            "--ros-args", 
            "--log-level", 
            "info",
        ],
    )
  
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )
    arm_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_controller"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    
    return LaunchDescription([
        move_group_node,
        #joint_state_broadcaster_spawner,
        #diff_drive_controller_spawner,
        #arm_joint_controller_spawner,
        rviz_node,
    ]) 
