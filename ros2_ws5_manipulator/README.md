## Basic manipulator using ros2 (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)

### basic urdf 

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description display.launch.py
    rqt_graph
    ros2 run tf2_tools view_frames

Use the following to debug urdf/xacro files:

    ros2 run xacro xacro my_robot.urdf.xacro 

## References:
- Edouard Renard, "ROS 2 Moveit 2 - Control a Robotic Arm" (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)

