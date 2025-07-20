## Basic manipulators using ros2 (<a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>) => Failed

| term | compositions | 
| - | - | 
| topic | publisher/subscriber | 
| service | client/server communicated through request/response |
| action | client/server communicated goal/feedback/result |

### general installation

Useful inspection code:

    ros2 pkg list

Launch ''CountUntil.action'' (WARNING: the first letter of the file name must be capitalized):

    sudo apt install ros-(...)-xacro     # if not installed
    mv ros2_ws4_manipulators ros2_ws4_manipulators_
    mkdir ros2_ws4_manipulators
    cd ros2_ws4_manipulators
    mkdir src
    colcon build
    cd src/
    ros2 pkg create arduino_firmware --build-type ament_python --dependencies rclpy
    cd ..
    # move everything from ros2_ws4_manipulators_ to ros2_ws4_manipulators
    colcon build --symlink-install
    source install/setup.bash

### communication to arduino

Upload ''src/arduino_firmware/firmware/serial_receiver_LED/serial_receiver_LED.ino'' to an arduino, then run the following:
    
    ros2 run arduino_firmware serial_publisher --ros-args -p port:=/dev/ttyACM0
    ros2 topic list
    ros2 topic pub /serial_transmitter example_interfaces/msg/String "data: '1'"

Upload ''src/arduino_firmware/firmware/serial_communicator/serial_communicator.ino'' to an arduino, then run the following:

    ros2 run arduino_firmware serial_lifecycle --ros-args -p port:=/dev/ttyACM0
    ros2 lifecycle nodes
    ros2 lifecycle set /serial_lifecycleNode configure
    ros2 lifecycle set /serial_lifecycleNode activate
    ros2 topic list
    ros2 topic echo /serial_lifecycle_receiver

Upload ''src/arduino_firmware/firmware/serial_transmitter/serial_transmitter.ino'' to an arduino, then run the following:

### course on urdf

    # Fixed Frame: world
    # add TF
    # add RobotModel: Description Topic: /robot_description
    ros2 launch robot_description rviz.launch.arduinobot.xacro.xml
    # or
    ros2 launch robot_description rviz.launch.arduinobot.xacro.py

### course on gazebo ros2_control
    sudo apt install ros-jazzy-ros2-control ros-jazzy-controller-manager ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-pluginlib
    source install/setup.bash
    # ros2 launch robot_build gazebo.rviz.launch.arduinobot.xacro.py
    ros2 launch robot_description gazebo.rviz.launch.arduinobot.xacro.py
    ros2 launch robot_controller ros2_control.launch.arduinobot.xacro.py

Had issue with ros2_control (<a href="https://github.com/ros-controls/gz_ros2_control/issues/567#issuecomment-2833271693">link</a>).

<span style="color:red"> => Failed to run chapter 5.51!</span>

This is <span style="color:red">red text</span> and <span style="color:blue">blue text</span>.

$${\color{red}=>\space Failed\space to\space run\space section \space 5.51!}$$	


### course on moveit

Installed moveit according to <a href="https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html">link</a>, using the ``main`` ``<branch>``. Don't forget to install Colcon with mixin. For Raspberry Pi 5 installation, two packages need to be skipped:

    colcon build --mixin release --packages-skip kortex_api kortex_driver

Instead of writing your own, can do the following:
    
    ros2 launch moveit_setup_assistant setup_assistant.launch.py


## References:
- Antonio Brandi, "Robotics and ROS 2 - Learn by Doing! Manipulators" (<a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>)

