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




## References:
- Edouard Renard, "ROS 2 - Hardware and ros2_control, Step by Step" (<a href="https://www.udemy.com/course/ros2_control/">Udemy</a>)

