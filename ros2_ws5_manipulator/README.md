## Basic manipulator using ros2 (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)

### basic urdf 

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description display.launch.py
    rqt_graph
    ros2 run tf2_tools view_frames

Use the following to debug urdf/xacro files:

    ros2 run xacro xacro my_robot.urdf.xacro 

### basic moveit2 setup assistance

    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    # Create New Moveit Configuration Package (or edit if the configuration files already exist)
    # Browse => src/my_robot_description/urdf/my_robot.urdf.xacro => Load Files
    # Start Screen: can toggle visual/collision
    # Self-Collisions => Generate Collision Matrix: removes all adjacent collisions and never in contact ones
    # Virtual Joints => Add Virtual Joint => Virtual Joint Name: virtual_join => Parent Frame Name: world => Joint Type: fixed => Save
    # Planning Groups => Add Group => Group Name: arm => Kinametic Solver: kdl_kinematics_plugin => Add Joints => choose all and right arrw => Save
    # Robot Poses => Add Pose => all joints at 0 => Pose Name: home => Save: can add a few other ones for debugging
    # ros2_control URDF Model => position for Command Interfaces and State Interfaces => Add interfaces
    # ROS2 Controllers => Auto Add JointTrajectoryController
    # Moveit Controllers => Auto Add FollowJointsTrajectory
    # Author Information => add anything (e.g. "my_robot", "my_robot@gmail.com"), otherwise bugged
    # Configuration Files => Browse: src/my_robot_bringup/ => Generate Package: double check if files are generated => Exit Setup Assistant

## References:
- Edouard Renard, "ROS 2 Moveit 2 - Control a Robotic Arm" (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)

