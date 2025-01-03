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
    source .../ROS_init_practice/install/setup.bash

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

### Basic simulation using URDF, rvis2, and gazebo
URDF: Unified Robot Description Format in XML format
- link: https://wiki.ros.org/urdf/XML/link
- joint: https://wiki.ros.org/urdf/XML/joint

Common URDF inspection code:

        ros2 run tf2_tools view_frames # after opening the model in rviz2

## References:
- msadowski, "awesome-weekly-robotics" (<a href="https://github.com/msadowski/awesome-weekly-robotics">GitHub</a>)
- Robotics Back-End, "ROS2 Tutorial - ROS2 Humble 2H50 [Crash Course]" (2022) (<a href="https://www.youtube.com/watch?v=Gg25GfA456o">YouTube</a>)

