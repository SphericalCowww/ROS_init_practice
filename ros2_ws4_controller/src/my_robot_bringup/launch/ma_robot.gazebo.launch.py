from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os

######################################################################################################################
def generate_launch_description():
    robot_description_path = get_package_share_path('my_robot_description')
    robot_bringup_path     = get_package_share_path('my_robot_bringup')
    
    urdf_path          = os.path.join(robot_description_path, 'urdf', 'ma_robot.urdf.xacro')
    gazebo_config_path = os.path.join(robot_bringup, 'config', 'gazebo_bridge.yaml')
    rviz_config_path   = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'ma_robot_controllers.yaml')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )
    # find physics plugins in: /opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ),
        ),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf --physics-engine libgz-physics7-bullet-featherstone-plugin"]),
        ],
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description"],
    )
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": gazebo_config_path}],
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
        arguments=["-d", rviz_config_path],
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        arm_joint_controller_spawner,
#        rviz_node,
    ])

######################################################################################################################
