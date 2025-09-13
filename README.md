# Learning the Robot Operating System 2 (ROS2)

Installed Ubuntu 24.04.1 LTS on Raspberry Pi 5. Before installing ROS2, watch out ``$VERSION_CODENAME`` or ``$UBUNTU_CODENAME`` can be empty. Replace it with the Ubuntu version's corresponding code name, e.g ''novel'' for 24.04. Or do,

    . /etc/os-release

and then install ROS2 following this <a href="https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">webpage</a>, and don't miss:

    sudo apt install python3-colcon-common-extensions

Installing common packages:

    sudo apt install ros-jazzy-xacro     
    sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
    sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-teleop-twist-keyboard
    sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
    colcon build --symlink-install

Installing gazebo simulator. Notice that jazzy uses "Gazebo Sim (Gazebo Harmonic) Plugin", instead of "<a href="https://classic.gazebosim.org/tutorials?tut=ros_gzplugins">Gazebo Classic Plugin</a>" like in humble:

    sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control 
    ros2 pkg list | grep gz                                              # make sure ros_gz_bridge is in the package list

Note that Raspberry Pi 5 may not be able to run gazebo using the ``orge2`` rendering engine. In this case, use ``orge`` by default instead:

    vim ~/.gz/sim/8/gui.config
    # change <engine>orge2</engine> to <engine>orge</engine>

Installing moveit2 manipulator (see <a href="https://moveit.ai/install-moveit2/binary/">webpage</a>):

    sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    sudo apt install ros-jazzy-moveit

Next, export the following in ``.bashrc``:

    export PS1='\u@\h:\W\$'                                             # this one is just shell formatting, not ros2 related
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp                        # fore moveit2      
    source /opt/ros/jazzy/setup.bash                                    # for ros2 
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash   # for colcon

Build a ROS package:

    mkdir (...)/ros2_ws(...)/
    cd (...)/ros2_ws(...)/
    colcon build
    source install/setup.bash                                         # make sure to run the correct setup.XXX, XXX shell type
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

