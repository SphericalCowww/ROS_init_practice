# Learning the Robot Operating System 2 (ROS2)

Installed Ubuntu 24.04.1 LTS on Raspberry Pi 5. Before installing ROS2, watch out ``$VERSION_CODENAME`` or ``$UBUNTU_CODENAME`` can be empty. Replace it with the Ubuntu version's corresponding code name, e.g ''novel'' for 24.04. Or do,

    . /etc/os-release
    sudo apt install python3-colcon-common-extensions

and then install ROS2 following this <a href="https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">webpage</a>. Export the following in ``.bashrc``:

    source /opt/ros/jazzy/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 
    source (...)/ros2_ws(...)/install/setup.bash              # optional

Build a ROS package with the following (python based): 

    mkdir (...)/ros2_ws(...)/
    cd (...)/ros2_ws(...)/
    colcon build
    cd src/
    # in python
    ros2 pkg create my_robot_descriptions --build-type ament_python --dependencies rclpy 
    # in cpp
    ros2 pkg create my_robot_controller --build-type ament_cmake --dependencies rclcpp

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

