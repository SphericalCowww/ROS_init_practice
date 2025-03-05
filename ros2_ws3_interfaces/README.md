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
    ros2 pkg create lifecycle_scripts --build-type ament_python --dependencies rclpy practice_robot_interfaces
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

Similarly, run the MoveDist action scripts:

    ros2 run action_scripts MoveDist_server 
    ros2 run action_scripts MoveDist_client 76 2
    ros2 run action_scripts MoveDist_client 0 0

Note that in ``../ROS_init_practice/ros2_ws3_interfaces/src/practice_robot_interfaces/action``, there are 3 types of variables in the action server:

| type | IO in code | description | 
| - | - | - |
| Goal | ServerGoalHandle.request | The ServerGoalHandle manages the state transitions of a goal, such as accepting, executing, succeeding, aborting, or canceling a goal. It ensures that each goal progresses through its lifecycle in a controlled manner. |
| Result | return parameter of ActionServer.execute_callback() | Upon completion of a goal, the ServerGoalHandle facilitates sending the final result back to the client, indicating the outcome of the action. |
| Feedback | ServerGoalHandle.publish_feedback() | It provides the publish_feedback() method, allowing the action server to send feedback to the client about the ongoing goal execution. This enables clients to receive real-time updates on the progress of their requested actions. |

From the server side, there are 2 ways to handle multiple callbacks/goals: linearly by ``MutuallyExclusiveCallbackGroup`` and in parallel by ''ReentrantCallbackGroup''. Watch out for the difference between the global ``ServerGoalHandle self.goal_handle_`` and the local ``ServerGoalHandle goal_handle``; they will be different when the server has multiple goals to handle.

From the client side, the ''send_goal_async'', ensures sending goals while spinning. The return object is a python ``concurrent.futures`` object (see <a href="https://www.youtube.com/watch?v=SAueUTQNup8">YouTube</a>).

Finally, there is a major structure difference between how the server and client codes use threads. For server, it directly has callback functions as input parameters, and uses ``ReentrantCallbackGroup()`` and ``threading.Lock()`` to pass down the callbacks. For client, it initializes callback functions via ''send_goal_async()'', and uses ``concurrent.futures`` and ``add_done_callback()`` to pass down the callbacks.

### lifecycle practice

Similarly, run the Number lifecycle script through ``lifecycle`` command lines:

    ros2 run lifecycle_scripts Number_publisher
    ros2 lifecycle nodes
    ros2 lifecycle list /Number_publisher
    ros2 lifecycle get /Number_publisher
    ros2 lifecycle set /Number_publisher configure/activate/deactivate/cleanup/shutdown
    ros2 topic list
    # at the activate state
    ros2 topic echo /number
    # while transitioning
    ros2 topic echo /Number_publisher/transition_event

Or it can be run with ``service`` command lines:

    ros2 service list
    ros2 service type /Number_publisher/get_state
    ros2 interface show lifecycle_msgs/srv/GetState
    ros2 service call /Number_publisher/get_state lifecycle_msgs/srv/GetState 
    ros2 service type /Number_publisher/change_state
    ros2 interface show lifecycle_msgs/srv/ChangeState
    ros2 service call /Number_publisher/change_state lifecycle_msgs/srv/ChangeState
    ros2 service call /Number_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: 'configure'}}"

Or it can be run with a Python script specifically written in ``lifecycle_scripts/Number_client.py``:
    
    ros2 run lifecycle_scripts Number_client --ros-args -p managed_node_name:="Number_publisher"

    colcon build --symlink-install
    ros2 launch practice_robot_interfaces practice_lifecycle.launch.xml
    ros2 launch practice_robot_interfaces practice_lifecycle.launch.py

## References:
- Edouard Renard, "ROS2 for Beginners Level 3 - Advanced Concepts" (<a href="https://www.udemy.com/course/ros2-advanced-core-concepts">Udemy</a>)
- rclpy developers, "Actions" (<a href="https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.server">webpage</a>)

