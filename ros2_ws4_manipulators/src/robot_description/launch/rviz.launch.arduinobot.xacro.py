import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
##############################################################################################################################
def generate_launch_description():
    ld = LaunchDescription()
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("robot_description"),
            "urdf",
            "arduinobot.rvis.urdf.xacro",
        ),
        description="absolute path to urdf file",
    )
   
    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            LaunchConfiguration("model"),
        ]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package   ="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    
    joint_state_publisher_gui = Node(
        package   ="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    rviz_node = Node(
        package   ="rviz2",
        executable="rviz2",
        name      ="rviz2",
        output    ="screen",
        arguments =["-d", 
                    os.path.join(
                        get_package_share_directory("robot_description"), 
                        "rviz2", 
                        "display.rviz",
                    )
        ],
    )

    ld.add_action(model_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz_node)

    return ld
