# Learning the Robot Operating System 2 (ROS2)

Installed Xcode with the following:

    sudo apt install software-properties-common apt-transport-https wget -y
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt update

Installed Ubuntu 24.04.1 LTS on Raspberry Pi 4. Installed ROS2 following this <a href="https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">webpage</a>. However, watch out ''$UBUNTU_CODENAME'' can be empty. Replace it with the Ubuntu version's corresponding code name, e.g ''novel'' for 24.04.

Build a ROS package with the following (python based): 

    mkdir (...)/ros2_ws(...)/
    cd (...)/ros2_ws(...)/
    colcon build
    cd src/
    ros2 pkg create (...project_name...) --build-type ament_python --dependencies rclpy

Exporting the following in ''.bashrc'':

    source /opt/ros/jazzy/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash # if not found: sudo apt install python3-colcon-common-extensions
    source (...)/ros2_ws(...)/install/setup.bash

Common ROS terminal commands:

    rqt_graph
    ros2 node\topic\service\param list\info\type\hz\echo
    ros2 interface show std_msgs/msg/String

## Terminologies:

- high level:
    * determine end-effector trajectory using control theory with variational method/functional analysis/Lagrange multiplier (<a href="https://www.amazon.de/-/en/Donald-Kirk-ebook/dp/B00CWR4MX0">book</a>)
- low level:
    * solving end-effector link/joint configuration using inverse kinematic algorithm (<a href="https://www.amazon.de/-/en/Bruno-Siciliano-ebook/dp/B007IDTLL6">book</a>)
    * Denavit–Hartenberg (DH) Convention: <a href="https://www.youtube.com/watch?v=rA9tm0gTln8">YouTube</a>

## References:
- msadowski, "awesome-weekly-robotics" (<a href="https://github.com/msadowski/awesome-weekly-robotics">GitHub</a>)
- D. E. Kirk, "Optimal Control Theory: An Introduction" (1998) (<a href="https://www.amazon.de/-/en/Donald-Kirk-ebook/dp/B00CWR4MX0">book</a>)
- B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, "Robotics: Modelling, Planning and Control (2012)" (<a href="https://www.amazon.de/-/en/Bruno-Siciliano-ebook/dp/B007IDTLL6">book</a>)

