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
    urdf_model = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("robot_description"),
            "urdf",
            "arduinobot.urdf.xacro",
        ),
        description="Absolute path to urdf file",
    )
    robot_state_publisher = Node(
        package   ="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    "xacro",
                    " ",
                    LaunchConfiguration("model"),
                ]),
                value_type=str,
            ),
            "use_sim_time":True,    
        }],
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
                    ),
        ],
    )
   
    gazebo_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(get_package_share_directory("robot_description")).parent.resolve()),
        ],
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
        arguments=["-topic", "robot_description",
                   "-name",  "arduinobot",
        ],
    )
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
    )

    '''
    ### same error, see: https://github.com/ros-controls/gz_ros2_control/issues/567
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
    '''

    ld.add_action(urdf_model)
    ld.add_action(robot_state_publisher)
    #ld.add_action(joint_state_publisher_gui)
    #ld.add_action(rviz_node)
    ld.add_action(gazebo_path)
    ld.add_action(gazebo)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)
    
    #ld.add_action(arm_controller_spawner)
    #ld.add_action(gripper_controller_spawner)
    #ld.add_action(joint_state_broadcaster_spawner)

    return ld
