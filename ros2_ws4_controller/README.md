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
    # copy the full my_robot_description directly
    ros2 pkg create my_robot_bringup
    ros2 pkg create my_robot_firmware --build-type ament_cmake
    ros2 pkg create my_robot_firmware_py --build-type ament_python --dependencies rclpy python3-adafruit-blinka
    cd ..
    # move everything from ros2_ws4_controller_ to ros2_ws4_controller
    colcon build --symlink-install
    source install/setup.bash

WARNING: do NOT ``source install/setup.sh``!

### basic urdf 

    ros2 launch my_robot_description display.launch.xml 
    rqt_graph
    ros2 run tf2_tools view_frames

Use the following to debug urdf/xacro files:

    ros2 run xacro xacro my_robot.urdf.xacro 

### controller configuration
Visti <a href="https://github.com/ros-controls/ros2_controllers/tree/jazzy/">ros2_controllers github</a> and <a href="https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller_parameter.yaml">diff drive controller parameter</a> to fill in the ros2_control configuration file:  ``.../src/my_robot_bringup/config/my_robot_controllers.yaml``.

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.launch.py
    # on a different window
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
    
Other parameters to check:
    
    rqt_graph
    ros2 control list_controllers
    ros2 control list_controller_types
    ros2 control list_hardware_interfaces
    ros2 control list_hardware_components

### controlling servo with controller PCA9685 without ros2_control

#### with an Arduino using the Arduino IDE
Install ``Adafruit PCA9685 PWM Servo Driver`` library in the Arduino IDE. Open the following file with the Arduino/PCA9685 connected as shown in the photo:

    #upload to Arduino: /src/my_robot_firmware/firmware/arduino_PCA9685controllerTestChannel0/PCA9685controllerTestChannel0.ino

<img src="https://github.com/SphericalCowww/ROS_init_practice/blob/main/ros2_ws4_controller/src/my_robot_firmware/firmware/Arduino_PCA9685_channel0.png" width="300">

#### with an Arduino using the ROS serial to communicate with it

ROS2 does NOT have an intrinsic package to communicate with an Arduino. To use ROS on an Arduino, a serial (I2C) connection needs to be established while connecting the Arduino to Rasp Pi on a USB port. To enable I2C:

    sudo apt update
    sudo apt upgrade
    sudo apt-get install raspi-config
    sudo raspi-config 	                #Navigate to Interfacing Options > I2C, and enable it.
    reboot
    sudo apt-get install -y i2c-tools python3-smbus
    i2cdetect -y 1                        #If I2C is enabled, it will show grid patterns
    sudo adduser $USER i2c

Then the communication can be established in the following ways.

Arduino as a transmitter and ROS as the receiver (very inconsistent on Rasp Pi):
    
    # upload to Arduino: src/arduino_firmware/firmware/serial_transmitter/serial_transmitter.ino
    # open Serial Monitor from Arduino IDE, wait for consistent messages, close Serial Monitor
    ros2 run my_robot_firmware_py Arduino_serial_receiver --ros-args -p port:=/dev/ttyACM0

Arduino as the receiver and ROS as the transmitter:

    # upload to Arduino: /src/my_robot_firmware/firmware/Arduino_serial_receiver_LED/Arduino_serial_receiver_LED.ino
    ros2 run my_robot_firmware_py Arduino_serial_publisher --ros-args -p port:=/dev/ttyACM0
    ros2 topic list
    ros2 topic pub /serial_transmitter example_interfaces/msg/String "data: '1'"     # turn on LED_PIN 13
    ros2 topic pub /serial_transmitter example_interfaces/msg/String "data: '0'"     # turn off LED_PIN 13

Arduino as the transmitter and receiver, and ROS as the lifecycle:

    # upload to Arduino: /src/my_robot_firmware/firmware/Arduino_serial_communicator/Arduino_serial_communicator.ino
    ros2 run my_robot_firmware_py Arduino_serial_lifecycle --ros-args -p port:=/dev/ttyACM0
    ros2 lifecycle nodes
    ros2 lifecycle set /serial_lifecycleNode configure
    ros2 lifecycle set /serial_lifecycleNode activate
    ros2 topic list
    ros2 topic echo /serial_lifecycle_receiver

Using the same circuit configuration as shown in the photo of the previous section. We can use Arduino as the receiver to control PCA9685, and ROS as the lifecycle to transmit the on/off signal:

    # upload to Arduino: /src/my_robot_firmware/firmware/Arduino_serial_communicator/Arduino_serial_receiver_PCA9685.ino
    ros2 run my_robot_firmware_py Arduino_serial_lifecycle --ros-args -p port:=/dev/ttyACM0
    ros2 lifecycle set /serial_lifecycleNode configure
    ros2 lifecycle set /serial_lifecycleNode activate
    ros2 topic pub /serial_lifecycle_transmitter example_interfaces/msg/String "data: '1'"
    ros2 lifecycle set /serial_lifecycleNode deactivate
    ros2 lifecycle set /serial_lifecycleNode cleanup

#### with the driver from python package adafruit_pca9685 (<a href="https://github.com/adafruit/Adafruit_CircuitPython_PCA9685">github</a>)

To install the python pakcage ``adafruit_pca9685``  on a raspPi, do 

    sudo apt-get install build-essential python3 python3-dev python3-venv python3-pip
    sudo apt install python3-lgpio
    pip install --break-system-packages adafruit-python-shell click wheel 
    pip install --break-system-packages adafruit-circuitpython-servokit adafruit-circuitpython-pca9685 Adafruit-Blinka adafruit-circuitpython-register adafruit-circuitpython-busdevice

And then follow the connection to the photo:

<img src="https://github.com/SphericalCowww/ROS_init_practice/blob/main/ros2_ws4_controller/src/my_robot_firmware/firmware/RaspPi_PCA9685_channel0.png" width="250">

Test the connection by:

    python3 src/my_robot_firmware_py/my_robot_firmware_py/testRaspPi5_adafruit_pca9685_channel0.py

To run in ROS, do:

    ros2 run my_robot_firmware_py RaspPi5_adafruit_lifecycle --ros-args -p port:=/dev/ttyACM0

#### with the driver from C++ package lib9685 (<a href="https://github.com/TeraHz/PCA9685/">github</a>)


## References:
- Edouard Renard, "ROS 2 - Hardware and ros2_control, Step by Step" (<a href="https://www.udemy.com/course/ros2_control/">Udemy</a>)

