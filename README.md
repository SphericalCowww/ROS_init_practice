# Learning the Robot Operating System 2 (ROS), Initial Practices

Installed Ubuntu 24.04.1 LTS on Raspberry Pi 4. Installed ROS2 following this <a href=" https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">webpage</a>. However, watch out ''$UBUNTU_CODENAME'' can be empty. Replace it with the Ubuntu version's corresponding code name, e.g ''novel'' for 24.04.

Installed Xcode with the following:

    sudo apt install software-properties-common apt-transport-https wget -y
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt update
    sudo apt install code

Exporting the following in ''.bashrc'':

    source /opt/ros/jazzy/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash # if not found: sudo apt install python3-colcon-common-extensions
    source ~/Documents/ROS_init_practice/install/setup.bash

Build a ROS package with the following (python based): 

    colcon build
    ros2 pkg create practice_controller --build-type ament_python --dependencies rclpy
    colcon build

Common ROS terminal commands:

    code . #open xcode
    rqt_graph
    ros2 node/topic/service node/list
    https://www.youtube.com/watch?v=Gg25GfA456o

