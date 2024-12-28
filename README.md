# Learning the Robot Operating System 2 (ROS), Initial Practices

Installed Xcode with the following:

    sudo apt install software-properties-common apt-transport-https wget -y
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt update
    sudo apt install code
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash # if not found: sudo apt install python3-colcon-common-extensions

Installed Ubuntu 24.04.1 LTS on Raspberry Pi 4. Installed ROS2 following this <a href=" https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">webpage</a>. However, watch out ''$UBUNTU_CODENAME'' can be empty. Replace it with the Ubuntu version's corresponding code name, e.g ''novel'' for 24.04.
    
Exporting the following in ''.bashrc'':

    source /opt/ros/jazzy/setup.bash
    source ~/Documents/ROS_init_practice/install/setup.bash

Build a ROS package with the following (python based): 

    colcon build
    ros2 pkg create practice_controller --build-type ament_python --dependencies rclpy

### Practice note

    cd ~/ROS_init_practice/
    colcon build --symlink-install
    ros2 run practice_controller test_node

## References:
- Robotics Back-End, "ROS2 Tutorial - ROS2 Humble 2H50 [Crash Course]" (2022) (<a href="https://www.youtube.com/watch?v=Gg25GfA456o">YouTube</a>)

