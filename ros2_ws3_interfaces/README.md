## Basic interfaces using ros2 (<a href="https://www.udemy.com/course/ros2-advanced-core-concepts">Udemy</a>)

| term | description | 
| - | - | 
| topic | the data stream of the robot | 
| node | a subcode that runs simultaneously from other nodes; need to start a new window to run each node |
| service | a single usually quick task performed by the topic via running several nodes |
| publisher/subscriber | like receiver/transmitter, messages are broadcast and received asynchronously, ideal for continuous data flow and decoupled communication |
| closed loop system | publisher + subscriber |
| client/server | one-to-one communication where the client waits for a response from the server, ideal for interactions that require a direct reply |
| state | condition of the system at a specific time; this condition can be in infinite loop until triggered |
| event | a trigger that results in the change of states |
| callback | a function that reacts to an event |
| feedback | communication monitoring of the states and events |
| state machine | a combination of states that can fully describe the system (see reference from <a href="https://github.com/SphericalCowww/Elec_FPGA_iCEstick_practice">GidHub</a>) |
| action | a state machine with feedbacks and conditions that tries to achieve a task goal with success or failure. Recommended using client/server |
| lifecycle | a sustainable state machine with initialization/configuration/activation/shutdown that can be commanded to do various tasks while live |
| cancel | a command that ends an event initiated by the action client |
| abort | an error that ends an event initiated by the action server |

### action practice

Useful inspection code:

    ros2 topic list --include-hidden-topic
    ros2 service list --include-hidden-service
    ros2 action list -t

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

Run the CountUntil action scripts (see <a href="https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.server">webpage</a> for available functions for Action Client/Server):
    
    ros2 interface show practice_robot_interfaces/action/CountUntil
    ros2 run action_scripts CountUntil_server 
    ros2 run action_scripts CountUntil_client                            # in a separate window
    # or in command line client node
    ros2 action send_goal /CountUntil practice_robot_interfaces/action/CountUntil "{target_number: 4, wait_time_per_count: 2}" --feedback 

    ros2 run action_scripts MoveDist_server 
    ros2 run action_scripts MoveDist_client 76 2
    ros2 run action_scripts MoveDist_client 0 0

### lifecycle practice

    ros2 run lifecycle lifecycle_scripts
    ros2 lifecycle nodes
    ros2 lifecycle list /Number_publisher
    ros2 lifecycle get /Number_publisher
    ros2 lifecycle set /Number_publisher configure/activate/deactivate/cleanup/shutdown
    ros2 topic echo /Number_publisher/transition_event

    ros2 service list
    ros2 service type /Number_publisher/get_state
    ros2 service type /Number_publisher/get_state
    ros2 interface show lifecycle_msgs/srv/GetState
    ros2 service type /Number_publisher/change_state
    ros2 interface show lifecycle_msgs/srv/ChangeState
    ros2 service call  /Number_publisher/change_state lifecycle_msgs/srv/ChangeState “{transition:{id: 1, label: ‘configure’}}”

    colcon build --symlink-install
    ros2 launch practice_robot_interfaces practice_lifecycle.launch.xml
    ros2 launch practice_robot_interfaces practice_lifecycle.launch.py

## References:
- Edouard Renard, "ROS2 for Beginners Level 3 - Advanced Concepts" (<a href="https://www.udemy.com/course/ros2-advanced-core-concepts">Udemy</a>)
- rclpy developers, "Actions" (<a href="https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.server">webpage</a>)

