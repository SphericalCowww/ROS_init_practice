## Basic controller using ros2 (<a href="https://www.udemy.com/course/ros2_control/">Udemy</a>)

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
    sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
    mv ros2_ws4_controller ros2_ws4_controller_
    mkdir ros2_ws4_controller
    cd ros2_ws4_controller
    mkdir src
    colcon build
    cd src/
    ros2 pkg create arduino_firmware --build-type ament_python --dependencies rclpy
    cd ..
    # move everything from ros2_ws4_controller_ to ros2_ws4_controller
    colcon build --symlink-install
    source install/setup.bash

### basic urdf 

    ros2 launch my_robot_description display.launch.xml 
    rqt_graph
    ros2 run tf2_tools view_frames

### controller configuration
Visti <a href="https://github.com/ros-controls/ros2_controllers/tree/jazzy/">ros2_controllers github</a> and <a href="https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller_parameter.yaml">diff drive controller parameter</a> to fill in the ros2_control configuration file:  ``.../src/my_robot_bringup/config/my_robot_controllers.yaml``.

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.launch.py
    rqt_graph

    ros2 control list_controllers
    ros2 control list_controller_types
    ros2 control list_hardware_interfaces
    ros2 control list_hardware_components

## References:
- Edouard Renard, "ROS 2 - Hardware and ros2_control, Step by Step" (<a href="https://www.udemy.com/course/ros2_control/">Udemy</a>)

