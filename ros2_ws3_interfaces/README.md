## Basic interfaces using ros2 (<a href="https://www.udemy.com/course/ros2-advanced-core-concepts">Udemy</a>)

| term | description | 
| - | - | 
| topic | the data stream of the robot | 
| node | a subcode that runs simultaneously from other nodes; need to start a new window to run each node |
| service | a single usually quick task performed by the topic via running several nodes |
| publisher/subscriber | like transmitter/receiver, messages are broadcast and received asynchronously, ideal for continuous data flow and decoupled communication |
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

### general installation

Useful inspection code:

    ros2 topic list --include-hidden-topic
    ros2 service list --include-hidden-service
    ros2 action list -t
    ros2 lifecycle nodes

Launch ''CountUntil.action'' (WARNING: the first letter of the file name must be capitalized):

    sudo apt install ros-(...)-xacro     # if not installed
    mv ros2_ws3_interfaces ros2_ws3_interfaces_
    mkdir ros2_ws3_interfaces
    cd ros2_ws3_interfaces
    mkdir src
    colcon build
    cd src/
    ros2 pkg create robot_interfaces
    ros2 pkg create robot_descriptions
    ros2 pkg create action_scripts --build-type ament_python --dependencies rclpy robot_interfaces
    ros2 pkg create lifecycle_scripts --build-type ament_python --dependencies rclpy robot_interfaces
    ros2 pkg create executor_scripts --build-type ament_python --dependencies rclpy robot_interfaces
    ros2 pkg create moveturtle_scripts --build-type ament_python --dependencies rclpy robot_interfaces
    cd ..
    # move everything from ros2_ws3_interfaces_ to ros2_ws3_interfaces
    colcon build --symlink-install
    source install/setup.bash

### action practice

Run the CountUntil action scripts (see <a href="https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.server">webpage</a> for available functions for Action Client/Server):
    
    ros2 interface show robot_interfaces/action/CountUntil
    ros2 run action_scripts CountUntil_server                            # in a separate window
    ros2 run action_scripts CountUntil_client                            
    # or in command line client node
    ros2 action send_goal /CountUntil robot_interfaces/action/CountUntil "{target_number: 4, wait_time_per_count: 2}" --feedback 

Similarly, run the MoveDist action scripts:

    ros2 run action_scripts MoveDist_server                              # in a separate window
    ros2 run action_scripts MoveDist_client MoveDist 76 2
    ros2 run action_scripts MoveDist_client MoveDist 0 0

Note that in ``../ROS_init_practice/ros2_ws3_interfaces/src/robot_interfaces/action``, there are 3 types of variables in the action server:

| type | IO in code | description | 
| - | - | - |
| Goal | ServerGoalHandle.request | The ServerGoalHandle manages the state transitions of a goal, such as accepting, executing, succeeding, aborting, or canceling a goal. It ensures that each goal progresses through its lifecycle in a controlled manner. |
| Result | return parameter of ActionServer.execute_callback() | Upon completion of a goal, the ServerGoalHandle facilitates sending the final result back to the client, indicating the outcome of the action. |
| Feedback | ServerGoalHandle.publish_feedback() | It provides the publish_feedback() method, allowing the action server to send feedback to the client about the ongoing goal execution. This enables clients to receive real-time updates on the progress of their requested actions. |

From the server side, there are 2 ways to handle multiple callbacks/goals: linearly by ``MutuallyExclusiveCallbackGroup`` and in parallel by ''ReentrantCallbackGroup''. Watch out for the difference between the global ``ServerGoalHandle self.goal_handle_`` and the local ``ServerGoalHandle goal_handle``; they will be different when the server has multiple goals to handle.

From the client side, the ''send_goal_async'', ensures sending goals while spinning. The return object is a python ``concurrent.futures`` object (see <a href="https://www.youtube.com/watch?v=SAueUTQNup8">YouTube</a>).

Finally, there is a major structure difference between how the server and client codes use threads. For the server, it directly has callback functions as input parameters, which uses ``ReentrantCallbackGroup()`` and ``threading.Lock()`` to pass down the callbacks. For the client, it initializes callback functions via ''send_goal_async()'', which uses ``concurrent.futures`` and ``add_done_callback()`` to pass down the callbacks.

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
    
    ros2 run lifecycle_scripts Number_manager --ros-args -p managed_node_name:="Number_publisher"

Or use the launch files at ``robot_interfaces/launch``:

    ros2 launch robot_descriptions Number_publisher.launch.xml
    ros2 launch robot_descriptions Number_publisher.launch.py

Similarly, run the MoveDist lifecycle scripts:

    ros2 run lifecycle_scripts MoveDist_lifecycleMulti                          # in a separate window
    ros2 lifecycle nodes
    ros2 lifecycle set /MoveDist_lifecycleMultiNode configure
    ros2 lifecycle set /MoveDist_lifecycleMultiNode activate
    ros2 action list -t
    ros2 run action_scripts MoveDist_client /MoveDist_lifecycleMultiNode_ACleg1 76 2

Or run the multiple MoveDist lifecycle node with predefined names:

    ros2 run lifecycle_scripts MoveDist_lifecycleMulti ros-args -r __node:LC_MoveDist1
    ros2 run lifecycle_scripts MoveDist_lifecycleMulti ros-args -r __node:LC_MoveDist2
    ros2 run action_scripts MoveDist_client /LC_MoveDist1_ACleg3 76 2

Or use the launch file at ``robot_interfaces/launch``:

    ros2 launch robot_descriptions MoveDist_lifecycleMulti.launch.xml
    ros2 run lifecycle_scripts MoveDist_manager --ros-args -p managed_node_names:="['LC_MoveDist1', 'LC_MoveDist2']"
    ros2 run action_scripts MoveDist_client /LC_MoveDist2_ACleg0 76 2

Or use this one launch file at ``robot_interfaces/launch``:

    ros2 launch robot_descriptions MoveDist_lifecycleMulti_fullProcess.launch.xml

### executor practice

Simply do:

    ros2 run executor_scripts Single_threaded_executor
    ros2 run executor_scripts Multi_threaded_executor

### turtlesim practice

To check the basic turtlesim functionality:

    ros2 run turtlesim turtlesim_node 
    ros2 service list
    ros2 service type /kill
    ros2 interface show turtlesim/srv/Kill
    ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle1'}"
    ros2 service type /spawn
    ros2 interface show turtlesim/srv/Spawn
    ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 2.0, theta: 1.0}”

To allow spawning, moving, and kill a turtle in an action goal of a lifecycle ``MoveTurtle_lifecycle``:

    ros2 run turtlesim turtlesim_node 
    ros2 run moveturtle_scripts MoveTurtle_lifecycle
    ros2 lifecycle nodes
    ros2 lifecycle set /MoveTurtle_lifecycleNode configure
    ros2 lifecycle set /MoveTurtle_lifecycleNode activate
    ros2 action list
    ros2 interface list | grep robot_interfaces/action/
    ros2 interface show robot_interfaces/action/MoveTurtle
    ros2 action send_goal /MoveTurtle robot_interfaces/action/MoveTurtle "{linear_vel_x: 5.0, angular_vel_z: 2.0, duration: 10.0}" --feedback 
    ros2 action send_goal /MoveTurtle robot_interfaces/action/MoveTurtle "{linear_vel_x: 20.0, angular_vel_z: 0.0, duration: 5.0}" --feedback 

Note: 

  * .xml launch file cannot easily do configure/activate, do .py launch files in the future after all
  * do NOT do tests in on_configure or on_activate of a lifecycle; the exception will be directly to on_error and not showing the actual compilation error
  * sometimes need to do configure/activate several times, especially with raspberry pi's
    
## References:
- Edouard Renard, "ROS2 for Beginners Level 3 - Advanced Concepts" (<a href="https://www.udemy.com/course/ros2-advanced-core-concepts">Udemy</a>)
- rclpy developers, "Actions" (<a href="https://docs.ros2.org/foxy/api/rclpy/api/actions.html#module-rclpy.action.server">webpage</a>)

