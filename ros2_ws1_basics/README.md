## Basic turtle controller using ros2 (<a href="https://www.youtube.com/watch?v=Gg25GfA456o">YouTube</a>)
ROS: Robot Operating System
- node: one run mode of the robot
- topic: main host code of the robot
- service: basically helps to change the configuration parameter of the topic

- publisher: transmitter
- subscriber: receiver
- closed loop system: transmitter + receiver

Node practice:

    mv ros2_ws1_basics ros2_ws1_basics_
    mkdir ros2_ws1_basics
    cd ros2_ws1_basics
    mkdir src
    colcon build
    cd src/
    ros2 pkg create practice_controller --build-type ament_python --dependencies rclpy robot_interfaces
    cd ..
    # move everything from ros2_ws1_basics_ to ros2_ws1_basics
    colcon build --symlink-install
    source install/setup.bash
    ros2 run practice_controller test_node

Publisher, subscriber, closed-loop system practice with ''turtlesim'' topic:

    ros2 run turtlesim turtlesim_node 
    ros2 run turtlesim turtle_teleop_key
    ros2 run practice_controller turtle_move_cycle_node 
    ros2 topic echo /turtle1/cmd_vel
    ros2 run practice_controller turtle_read_pose_node 
    ros2 run practice_controller turtle_controller_node
    
Service practice:

    ros2 service list
    ros2 service type /clear
    ros2 service call /clear std_srvs/srv/Empty 
    ros2 topic hz /turtle1/pose

The main code is covered in ''turtle_controller_node.py'', resulting in the trajectory and color change of ''turtlesim'' shown in the following plot:

<img src="https://github.com/SphericalCowww/ROS_init_practice/blob/main/ros2_ws1_basics/practice_controller_demo.png" width="400">
    
## References:
- Robotics Back-End, "ROS2 Tutorial - ROS2 Humble 2H50 [Crash Course]" (2022) (<a href="https://www.youtube.com/watch?v=Gg25GfA456o">YouTube</a>)

