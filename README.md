# Learning the Robot Operating System 2 (ROS), Initial Practices

Installed Xcode with the following:

    sudo apt install software-properties-common apt-transport-https wget -y
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt update
    sudo apt install code

Installed Ubuntu 24.04.1 LTS on Raspberry Pi 4. Installed ROS2 following this <a href="https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">webpage</a>. However, watch out ''$UBUNTU_CODENAME'' can be empty. Replace it with the Ubuntu version's corresponding code name, e.g ''novel'' for 24.04.

Build a ROS package with the following (python based): 

    colcon build
    cd src/
    ros2 pkg create practice_controller --build-type ament_python --dependencies rclpy

Exporting the following in ''.bashrc'':

    source /opt/ros/jazzy/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash # if not found: sudo apt install python3-colcon-common-extensions
    source (...)/ros2_ws(...)/install/setup.bash

Common ROS terminal commands:

    rqt_graph
    ros2 node\topic\service\param list\info\type\hz\echo
    ros2 interface show std_msgs/msg/String

### Basic turtle controller using ros2 (<a href="https://www.youtube.com/watch?v=Gg25GfA456o">YouTube</a>)
ROS: Robot Operating System
- node: one run mode of the robot
- topic: main host code of the robot
- service: basically helps to change the configuration parameter of the topic

- publisher: transmitter
- subscriber: receiver
- closed loop system: transmitter + receiver

Node practice:

    cd .../ROS_init_practice/
    colcon build --symlink-install
    ros2 run practice_controller test_node

Publisher, subscriber, closed-loop system practice with ''turtlesim'' topic:

    ros2 run turtlesim turtlesim_node 
    ros2 run turtlesim turtle_teleop_key
    ros2 run practice_controller turtle_move_cycle_node 
    ros2 topic echo /turtle1/cmd_vel
    ros2 run practice_controller turtle_read_pose_node 
    ros2 run practice_controller turtle_controller_node
    
Service practice:

    ros2 service list
    ros2 service type /clear
    ros2 service call /clear std_srvs/srv/Empty 
    ros2 topic hz /turtle1/pose

Actually, the main code is already covered in ''turtle_controller_node.py'', resulting in the trajectory and color change of ''turtlesim'' shown in the following plot:

<img src="https://github.com/SphericalCowww/ROS_init_practice/blob/main/practice_controller_demo.png" width="400">

### Basic simulation using URDF, rvis2, and gazebo (<a href="https://www.udemy.com/course/ros2-tf-urdf-rviz-gazebo">Udemy</a>)
URDF: Unified Robot Description Format in XML format
- link: https://wiki.ros.org/urdf/XML/link
- joint: https://wiki.ros.org/urdf/XML/joint

Common URDF inspection code:

    check_urdf practice.urdf
    ros2 run tf2_tools view_frames # after opening the model in rviz2

Launch ''practice.urdf'' in rviz2:

    sudo apt install ros-(...)-xacro     # if not installed
    cd ros2_ws2_vis_sim
    mkdir src/
    colcon build --symlink-install
    cd src/
    ros2 pkg create practice_robot_description
    cd practice_robot_description
    rm -r include/ src/
    mkdir urdf launch
    # move in ''urdf/practice.urdf'' and ''launch/display.launch.xml'', replace ''CMakeLists.txt''
    cd ros2_ws2_vis_sim
    colcon build
    file (...)/ros2_ws2_vis_sim/install/practice_robot_description/share/practice_robot_description/urdf/practice.urdf
    source install/setup.bash
    ros2 launch practice_robot_description display.launch.xml

In rvis2 do:

    # Fixed Frame: base_footprint or base_link
    # add RobotModel: Description: /robot_description
    # add TF
    # save practice.rviz in urdf/
    ros2 run rviz2 rviz2 -d (...)/ros2_ws2_vis_sim/install/practice_robot_description/share/practice_robot_description/practice.rviz
    # or
    ros2 launch practice_robot_description display.launch_withRvizConfig.xml

Updated with xacro:

    cd ros2_ws2_vis_sim/src/practice_robot_description
    # move items in ''urdf/'', ''launch/'', and ''mesh/'' accordingly
    colcon build --symlink-install
    ros2 launch practice_robot_description display.launch.practice.xacro.xml
    # or 
    ros2 launch practice_robot_description display.launch_withRvizConfig.practice.xacro.xml
    ros2 param get /robot_state_publisher robot_description # to check the parameter value calculated by xacro

Run gazebo:
    
    sudo apt install ros-<distro>-ros-gz   # only for Gazebo Harmonic on Ubuntu 24.04 and using ROS 2 Jazzy
    
## References:
- msadowski, "awesome-weekly-robotics" (<a href="https://github.com/msadowski/awesome-weekly-robotics">GitHub</a>)
- Robotics Back-End, "ROS2 Tutorial - ROS2 Humble 2H50 [Crash Course]" (2022) (<a href="https://www.youtube.com/watch?v=Gg25GfA456o">YouTube</a>)
- Edouard Renard, "ROS2 for Beginners Level 2 - TF | URDF | RViz | Gazebo" (<a href="https://www.udemy.com/course/ros2-tf-urdf-rviz-gazebo">Udemy</a>)

