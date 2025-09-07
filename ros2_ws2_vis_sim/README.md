## Basic simulation using URDF, rvis2, and gazebo (<a href="https://www.udemy.com/course/ros2-tf-urdf-rviz-gazebo">Udemy</a>)
URDF: Unified Robot Description Format in XML format
- link: https://wiki.ros.org/urdf/XML/link
- joint: https://wiki.ros.org/urdf/XML/joint

Common URDF inspection code:

    check_urdf diffCart.urdf
    ros2 run tf2_tools view_frames # after opening the model in rviz2

Launch ''diffCart.urdf'' in rviz2:

    sudo apt install ros-jazzy-xacro     # if not installed
    colcon build
    file install/practice_robot_description/share/practice_robot_description/urdf/diffCart.urdf
    source install/setup.bash
    ros2 launch practice_robot_description display.launch.diffCart.xml

In rvis2 do:

    # Fixed Frame: base_footprint or base_link
    # add TF
    # add RobotModel: Description Topic: /robot_description
    # save diffCart.rviz in urdf/
    ros2 run rviz2 rviz2 -d (...)/ros2_ws2_vis_sim/install/practice_robot_description/share/practice_robot_description/diffCart.rviz
    # or
    ros2 launch practice_robot_description display.launch_withRvizConfig.diffCart.xml

Updated with xacro:

    cd ros2_ws2_vis_sim/src/practice_robot_description
    # move items in ''urdf/'', ''launch/'', and ''mesh/'' accordingly
    colcon build --symlink-install
    ros2 launch practice_robot_description display.launch.diffCart.xacro.xml
    # or 
    ros2 launch practice_robot_description display.launch_withRvizConfig.diffCart.xacro.xml
    ros2 param get /robot_state_publisher robot_description # to check the parameter value calculated by xacro

Installing gazebo in jazzy. Notice that Jazzy uses "Gazebo Sim (Gazebo Harmonic) Plugin", instead of "<a href="https://classic.gazebosim.org/tutorials?tut=ros_gzplugins">Gazebo Classic Plugin</a>" like in Humble. Make sure ``ros_gz_bridge`` is in the package list:

    sudo apt install ros-jazzy-ros-gz
    sudo apt install ros-jazzy-joint-state-publisher
    sudo apt install ros-jazzy-joint-state-publisher-gui
    sudo apt install ros-jazzy-teleop-twist-keyboard
    sudo apt install ros-jazzy-ros2-control
    sudo apt install ros-jazzy-ros2-controllers
    sudo apt install ros-jazzy-gazebo-ros2-control 
    ros2 pkg list | grep gz

Launch ''diffCart.gazebo.urdf'' in gazebo. The kinematics uses the <a href="https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.hh">differential drive plugin</a>, and the Jazzy format following <a href="https://www.youtube.com/watch?v=9sjTrpxtBaE">YouTube</a>:
    
    cd ros2_ws2_vis_sim/src/practice_robot_description
    # move items in ''urdf/'', ''launch/'', ''config'', and ''mesh/'' accordingly
    colcon build --symlink-install
    ros2 launch practice_robot_description display.launch.diffCart.jazzy.gazebo.xacro.xml
    # to check geometry details within Gazebo: Entity Tree => diffCart => right click => view
    # to save the world in an .sdf file: Save world as... => add path => OK
    # to load the ros2_ws2_vis_sim/src/practice_robot_description/urdf/diffCart.sdf file at launch:
    ros2 launch practice_robot_description display.launch_withSDFworld.diffCart.jazzy.gazebo.xacro.xml 
    
To issue control through the command line:

    ros2 topic list
    ros2 topic info /cmd_vel
    ros2 interface show geometry_msgs/msg/Twist
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0}}"

To issue interactive control:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Finally:

    ros2 topic echo /odom # to see if the control is odometry kinematics is correctly updating
    ps -aux | grep gz # notice the servers may still be running even after quitting the program

<img src="https://github.com/SphericalCowww/ROS_init_practice/blob/main/ros2_ws2_vis_sim/diffCart_gazebo_demo.png" width="1000">

Additionally, a robot arm with physics is added to the cart:

    ros2 launch practice_robot_description display.launch.diffCartWithArm.jazzy.gazebo.xacro.xml 

However, the controller is missing. For Jazzy, the use of ''ros2_control'' will be required and will need to be updated later for the learning will be quite in-depth; see <a href="https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html">Tutorial</a> and <a href="https://www.reddit.com/r/ROS/comments/161s6cv/to_ros2_control_or_to_not_ros2_control/">Reddit</a>.
    
## References:
- Edouard Renard, "ROS2 for Beginners Level 2 - TF | URDF | RViz | Gazebo" (<a href="https://www.udemy.com/course/ros2-tf-urdf-rviz-gazebo">Udemy</a>)
- Open Source Robotics Foundation, "DiffDrive.hh" (<a href="https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.hh">GitHub code</a>)
- Aleksandar Haber PhD, "Model, Simulate, and Control Differential Drive Robot in ROS2 Jazzy and Gazebo Harmonic from Scratch" (<a href="https://www.youtube.com/watch?v=9sjTrpxtBaE">YouTube</a>)
- ros2_control Development Team, "ros2_control demo", (<a href="https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html">Tutorial</a>)

