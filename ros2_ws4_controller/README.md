## Basic controller using ros2 (<a href="https://www.udemy.com/course/ros2_control/">Udemy</a>)

| term | compositions | 
| - | - | 
| topic | publisher/subscriber | 
| service | client/server communicated through request/response |
| action | client/server communicated goal/feedback/result |

### basic urdf 

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description display.launch.py
    # toggle for visualization RobotModel => Collision Enabled

Use the following to debug urdf/xacro files:

    ros2 run xacro xacro my_robot.urdf.xacro 

### controller configuration
Visit <a href="https://github.com/ros-controls/ros2_controllers/tree/jazzy/">ros2_controllers github</a> and <a href="https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/src/diff_drive_controller_parameter.yaml">diff drive controller parameter</a> to fill in the ros2_control configuration file:  ``src/my_robot_bringup/config/my_robot_controllers.yaml``.

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

    #upload to Arduino: src/my_robot_firmware/firmware/arduino_PCA9685controllerTestChannel0/PCA9685controllerTestChannel0.ino

<img src="https://github.com/SphericalCowww/ROS_init_practice/blob/main/ros2_ws4_controller/src/my_robot_firmware/firmware/Arduino_PCA9685_channel0.png" width="300">

#### with an Arduino using the ROS serial to communicate with it

ROS2 does NOT have an intrinsic package to communicate with an Arduino. To use ROS on an Arduino, a serial (I2C) connection needs to be established while connecting the Arduino to Rasp Pi on a USB port. To enable I2C:

    sudo apt update
    sudo apt upgrade
    sudo apt-get install raspi-config
    sudo raspi-config 	                  # navigate to Interfacing Options > I2C, and enable it.
    reboot
    sudo apt-get install -y i2c-tools python3-smbus
    i2cdetect -y 1                        # if I2C is enabled, it will show grid patterns
    sudo adduser $USER i2c
    colcon build
    source install/setup.bash

Then the communication can be established in the following ways.

Arduino as a transmitter and ROS as the receiver (very inconsistent on Rasp Pi):
    
    # upload to Arduino: src/arduino_firmware/firmware/serial_transmitter/serial_transmitter.ino
    # open Serial Monitor from Arduino IDE, wait for consistent messages, close Serial Monitor
    ros2 run my_robot_firmware_py Arduino_serial_receiver --ros-args -p port:=/dev/ttyACM0

Arduino as the receiver and ROS as the transmitter:

    # upload to Arduino: src/my_robot_firmware/firmware/Arduino_serial_receiver_LED/Arduino_serial_receiver_LED.ino
    ros2 run my_robot_firmware_py Arduino_serial_publisher --ros-args -p port:=/dev/ttyACM0
    ros2 topic list
    ros2 topic pub /serial_transmitter example_interfaces/msg/String "data: '1'"     # turn on LED_PIN 13
    ros2 topic pub /serial_transmitter example_interfaces/msg/String "data: '0'"     # turn off LED_PIN 13

Arduino as the transmitter and receiver, and ROS as the lifecycle:

    # upload to Arduino: src/my_robot_firmware/firmware/Arduino_serial_communicator/Arduino_serial_communicator.ino
    ros2 run my_robot_firmware_py Arduino_serial_lifecycle --ros-args -p port:=/dev/ttyACM0
    ros2 lifecycle nodes
    ros2 lifecycle set /serial_lifecycleNode configure
    ros2 lifecycle set /serial_lifecycleNode activate
    ros2 topic list
    ros2 topic echo /serial_lifecycle_receiver

Using the same circuit configuration as shown in the photo of the previous section. We can use Arduino as the receiver to control PCA9685, and ROS as the lifecycle to transmit the on/off signal:

    # upload to Arduino: src/my_robot_firmware/firmware/Arduino_serial_communicator/Arduino_serial_receiver_PCA9685.ino
    ros2 run my_robot_firmware_py Arduino_serial_lifecycle --ros-args -p port:=/dev/ttyACM0
    ros2 lifecycle set /serial_lifecycleNode configure
    ros2 lifecycle set /serial_lifecycleNode activate
    ros2 topic pub /serial_lifecycle_transmitter example_interfaces/msg/String "data: '1'"
    ros2 lifecycle set /serial_lifecycleNode deactivate
    ros2 lifecycle set /serial_lifecycleNode cleanup

#### with the driver from python package adafruit_pca9685 

To install the python pakcage ``adafruit_pca9685`` on a raspPi (<a href="https://github.com/adafruit/Adafruit_CircuitPython_PCA9685">github</a>), do 

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

#### with the driver from C++ package lib9685 >>> IMPORTANT FOR THE REST

  * pure driver: <a href="https://github.com/TeraHz/PCA9685/">github</a>, <a href="https://github.com/TeraHz/I2C/">github</a>
  * ros2 node: <a href="https://github.com/kimsniper/ros2_pca9685">github</a>, <a href="https://github.com/vertueux/i2c_pwm_board">github</a>
  * ros2_control node: <a href="https://github.com/rosblox/pca9685_ros2_control">github</a>, <a href="https://discourse.openrobotics.org/t/ros2-control-hardware-interface-for-adafruit-16-channel-pwm-servo-bonnet-for-raspberry-pi-pca9685/31772">discussion</a>

Proceeding with the pure drive option, which needs I2C as well:

    sudo apt update
    sudo apt upgrade
    sudo apt-get install build-essential libi2c-dev i2c-tools
    sudo apt-get install raspi-config
    sudo raspi-config 	                # navigate to Interfacing Options > I2C, and enable it.
    reboot
    ls /dev/i2c-*                       # if I2C is enabled, it will should show /dev/i2c-1
    sudo i2cdetect -y 1                 # if I2C is enabled, it will show grid patterns with 40 as the default address
    sudo usermod -aG i2c $USER          # lower level than "sudo adduser $USER i2c"
    reboot
    colcon build
    source install/setup.bash

Then move the ``I2C.h``, ``I2C.cpp``, and ``PCA9685.h`` from <a href="https://github.com/TeraHz/PCA9685/">github</a>, <a href="https://github.com/TeraHz/I2C/">github</a> to ``src/my_robot_firmware/include/pca9685``. Note, don't copy ``PCA9685.cpp``, it has been corrected by <a href="https://aistudio.google.com">Google AI Studio</a>. Then run,

    ros2 run my_robot_firmware testRaspPi5_pca9685_channel0 
    ros2 run my_robot_firmware testRaspPi5_pca9685_channel01

### ros2_control with one plugin (Ch4)

Connect the servos to channels 0 and 1 of the PCA9685. Note, however, that the RC servos used do not have speed mode and feedback, so remove the read() requirement in ``src/my_robot_firmware/src/hardware_interface_pca9685.cpp`` and mimic the differential drive with angular position control instead of speed control. Run:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.launch.py
    # on a different window
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

Note that errors will occur if the PCA9685 is not powered on. Also, switch ``src/my_robot_description/urdf/mobile_base.ros2_control.xacro`` to two line beginning with ``<!--plugin>mock_components/GenericSystem</plugin>`` to run with ros2_control without interacting with the hardware.

The launch reference goes like this

  * src/my_robot_**bringup**/launch/**my_robot.launch.py**
    * src/my_robot_description/urdf/my_robot.urdf.xacro
      * src/my_robot_**description**/urdf/**mobile_base.ros2_control.xacro**
        * src/my_robot_**firmware**/include/my_robot_firmware/**hardware_interface_pca9685_base.hpp**
          * src/my_robot_**firmware**/include/pca9685/**PCA9685.h**
            * src/my_robot_firmware/include/pca9685/I2C.h
      * src/my_robot_description/urdf/mobile_base.xacro
      * src/my_robot_description/urdf/common_properties.xacro
    * src/my_robot_**bringup**/config/**my_robot_controllers.yaml**
      * joint_state_broadcaster/JointStateBroadcaster
      * **diff_drive_controller/DiffDriveController**
        * src/my_robot_description/urdf/mobile_base.xacro

### ros2_control with two plugins (Ch5)
Connect the servos to channels 0-3 of the PCA9685. It uses both ``ros2_control`` classes <a href="https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/">diff_drive_controller</a> and <a href="https://github.com/ros-controls/ros2_controllers/tree/jazzy/forward_command_controller">forward_command_controller</a>. It says that in order to use ``move_it`` later, should also checkout <a href="https://github.com/ros-controls/ros2_controllers/tree/jazzy/joint_trajectory_controller">joint_trajectory_controller</a>. Run:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup ma_robot.launch.py
    # on a different window
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
    # on a different window
    ros2 topic pub -1 /arm_joint_controller/commands std_msgs/msg/Float64MultiArray "{data: [4.0, 3.0]}"
    ros2 topic pub -1 /arm_joint_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
    
    ros2 topic list
    ros2 topic info /arm_joint_controller/commands
    ros2 interface show std_msgs/msg/Float64MultiArray
    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 control list_hardware_components

Note that errors will occur if the PCA9685 is not powered on. Also, switch ``src/my_robot_description/urdf/mobile_base.ros2_control.xacro`` and/or ``src/my_robot_description/urdf/arm.ros2_control.xacro`` to two line beginning with ``<!--plugin>mock_components/GenericSystem</plugin>`` to run with ros2_control without interacting with the hardware. Any combination should work.

#### ros2_control with Gazebo

Note that after launching ``ros2 launch my_robot_bringup my_robot.launch.py``, collision and inertia geometries can be found under ``RobotModel``. Then simply change to this launch line:

    ros2 launch my_robot_bringup ma_robot.gazebo.launch.py

Sometimes it takes a few tries until the GUI is available and all the controllers are linked.

### ros2_control with locally written controller (Ch6)

Connect the servos to channels 0-3 of the PCA9685. It uses both ``ros2_control`` classes <a href="https://github.com/ros-controls/ros2_controllers/blob/jazzy/diff_drive_controller/">diff_drive_controller</a> and the controller ``src/my_robot_controller/include/my_robot_controller/me_controller.hpp``. 

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup me_robot.launch.py
    # on a different window
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
    # on a different window
    ros2 topic list
    ros2 topic info /joints_command
    ros2 topic pub -1 /my_arm_controller/joints_command example_interfaces/msg/Float64MultiArray "{data: [0.1, 0.1]}"
    ros2 topic pub -1 /my_arm_controller/joints_command example_interfaces/msg/Float64MultiArray "{data: [-0.1, -0.1]}"

### ros2_control Gazebo with MoveIt2 (Extra)

Run the MoveIt2 to be linked with Gazebo by:

    ros2 launch my_robot_bringup ma_robot.gazebo.launch.py
    # on a different window
    ros2 launch my_robot_bringup mi_robot.moveit.launch.py
    # Add => moveit_ros_visualization/MotionPlanning => ompl => Approx IK Solutions => (dragging the sphere around) => Plan & Execute

Note the following error is fine, just without sense:

    [move_group-1] [ERROR] [1757189711.578782510] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates 

### Extra notes

| term | description | configuration | interrupt handling | MoveIt compatibility |
| - | - | - | - | - |
| forward_command_controller | Sends direct commands (position, velocity, or effort) to a joint or set of joints without trajectory interpolation | simple configuration parameters | immediate overwrite | does work with MoveIt |
| joint_trajectory_controller | Executes full joint trajectories over time. It interpolates between trajectory points, manages timing, and handles smooth motion for multiple joints simultaneously | full PID specification | smooth blending | preferred by MoveIt |

## References:
- Edouard Renard, "ROS 2 - Hardware and ros2_control, Step by Step" (<a href="https://www.udemy.com/course/ros2_control/">Udemy</a>)
- Antonio Brandi, "Robotics and ROS 2 - Learn by Doing! Manipulators" (<a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>)
