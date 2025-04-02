## Basic manipulators using ros2 (<a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>)

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

    ros2 run arduino_firmware serial_receiver --ros-args -p port:=/dev/ttyACM0
    ros2 topic list
    ros2 topic echo /serial_receiver 

## References:
- Antonio Brandi, "Robotics and ROS 2 - Learn by Doing! Manipulators" (<a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>)

