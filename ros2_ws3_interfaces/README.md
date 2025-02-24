## Basic interfaces using ros2 (<a href="https://www.udemy.com/course/ros2-advanced-core-concepts">Udemy</a>)
ROS: Robot Operating System
- topic: the data stream of the robot
- node: a subcode that runs simultaneously from other nodes; need to start a new window to run each node
- service: a single usually quick task performed by the topic via running several nodes
- publisher/subscriber: like receiver/transmitter, messages are broadcast and received asynchronously, ideal for continuous data flow and decoupled communication
- closed loop system: publisher + subscriber
- client/server: one-to-one communication where the client waits for a response from the server, ideal for interactions that require a direct reply
- action: a state machine with feedbacks and conditions that tries to achieve a task goal with success or failure. Recommended using client/server

Launch ''CountUntil.action'' (WARNING: the first letter of the file name must be capitalized):

    sudo apt install ros-(...)-xacro     # if not installed
    mv ros2_ws3_interfaces ros2_ws3_interfaces_
    mkdir ros2_ws3_interfaces
    cd ros2_ws3_interfaces
    mkdir src
    colcon build --symlink-install
    cd src/
    ros2 pkg create practice_robot_interfaces
    ros2 pkg create action_scripts --build-type ament_python --dependencies rclpy practice_robot_interfaces
    cd ..
    # move everything from ros2_ws3_interfaces_ to ros2_ws3_interfaces
    colcon build --packages-select practice_robot_interfaces
    colcon build --packages-select action_scripts --symlink-install
    source install/setup.bash
    
    ros2 interface show practice_robot_interfaces/action/CountUntil
    ros2 run action_scripts CountUntil_server 
    ros2 run action_scripts CountUntil_client                            # in a separate winder


## References:
- Edouard Renard, "ROS2 for Beginners Level 3 - Advanced Concepts" (<a href="https://www.udemy.com/course/ros2-advanced-core-concepts">Udemy</a>)

