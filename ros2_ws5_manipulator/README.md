## Basic manipulator using ros2 (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)

### basic urdf 

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description display.launch.py
    rqt_graph
    ros2 run tf2_tools view_frames

Use the following to debug urdf/xacro files:

    ros2 run xacro xacro my_robot.urdf.xacro 

### moveit2 setup assistance with an arm

Launch the MoveIt assistance:

    sudo apt update
    sudo apt install
    colcon build
    source install/setup.bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    # Create New Moveit Configuration Package (or edit if the configuration files already exist)
    # Browse => src/my_robot_description/urdf/my_robot.urdf.xacro => Load Files
    # Start Screen: can toggle visual/collision
    # Self-Collisions => Generate Collision Matrix: removes all adjacent collisions and never in contact ones
    # Virtual Joints => Add Virtual Joint => Virtual Joint Name: virtual_join => Parent Frame Name: world => Joint Type: fixed => Save
    # Planning Groups => Add Group => Group Name: arm => Kinametic Solver: kdl_kinematics_plugin => Add Joints => choose all and right arrow => Save
    # Robot Poses => Add Pose => all joints at 0 => Pose Name: home => Save: can add a few other ones for debugging
    # ros2_control URDF Model => position for Command Interfaces and State Interfaces => Add interfaces
    # ROS2 Controllers => Auto Add JointTrajectoryController
    # Moveit Controllers => Auto Add FollowJointsTrajectory
    # Author Information => add anything (e.g. "my_robot", "my_robot@gmail.com"), otherwise bugged
    # Configuration Files => Browse: src/my_robot_moveit_config/ => Generate Package: double check if files are generated => Exit Setup Assistant

Fix the following file:

    # src/my_robot_moveit_config/config/joint_limits.yaml => max_velocity: 1.0, has_acceleration_limits: true, max_acceleration: 1.0 (need to be float)
    # src/my_robot_moveit_config/config/moveit_controllers.yaml => add in arm_controller: 
    ## action_ns: follow_joint_trajectory
    ## default: true

Launch the demo:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_moveit_config demo.launch.py
    # ignore: [move_group-3] [ERROR] [1758361830.007872451] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
    # ignore: [rviz2-4] [ERROR] [1758361834.128908606] [moveit_143394722.moveit.ros.motion_planning_frame]: Action server: /recognize_objects not available
    # MotionPlanning:
    ## Planning Group: arm
    ## Goal State: pose1
    ## Plan
    ## Execute

Can also try moving the 3D model (slowly if on rasp pi) before Plan and Execute. Also, try selecting ``MotionPlanning:Use Cartesian Path``, where planning may fail due to the solution not existing.

### moveit2 setup assistance with an arm and gripper

Launch the MoveIt assistance:

    sudo apt update
    sudo apt install
    colcon build
    source install/setup.bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    # Create New Moveit Configuration Package (or edit if the configuration files already exist)
    # Browse => src/my_robot_description/urdf/ma_robot.urdf.xacro => Load Files
    # ...
    # Planning Groups => Add Group => Group Name: gripper => Kinametic Solver: kdl_kinematics_plugin => Add Joints => choose gripper joint and right arrow => Save
    # End Effectors => End Effector Name: gripper_end_effector => End Effector Group: gripper => Parent Lin: tool_link => Parent Group: arm => Save
    # ...
    
Fix the following file:

    # src/ma_robot_moveit_config/config/joint_limits.yaml => max_velocity: 1.0, has_acceleration_limits: true, max_acceleration: 1.0 (need to be float)
    # src/ma_robot_moveit_config/config/moveit_controllers.yaml => add in gripper_controller/arm_controller: 
    ## action_ns: follow_joint_trajectory
    ## default: true

Launch the demo:

    colcon build
    source install/setup.bash
    ros2 launch ma_robot_moveit_config demo.launch.py
    # ignore: [move_group-3] [ERROR] [1758361830.007872451] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
    # ignore: [rviz2-4] [ERROR] [1758361834.128908606] [moveit_143394722.moveit.ros.motion_planning_frame]: Action server: /recognize_objects not available
    # optional: MotionPlanning => Planned Path => Loop Animation: off

Note that setting the gripper as an end effector is necessary. Otherwise, if a gripper piece is used as the pose goal, the inverse-kinematic solve for the arm movement would have to account for tiny changes in gripper position, which adds non-essential degrees of freedom and makes convergence much harder. By using the end effector base link as the reference, the arm can be much cleanly planned, and the gripper action remains independent of the arm action.

### local launch file for moveit2
    
    cp src/ma_robot_moveit_config/config/ros2_controllers.yaml src/my_robot_bringup/config/ma_robot_controllers.yaml
    cp src/ma_robot_moveit_config/config/ma_robot.ros2_control.xacro src/my_robot_description/urdf/
    # modify ma_robot.ros2_control.xacro accordingly
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup ma_robot.launch.py
    # ignore: [move_group-3] [ERROR] [1758361830.007872451] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
    # Add => MotionPlanning
    # Context => Planning Library => ompl
    
## References:
- Edouard Renard, "ROS 2 Moveit 2 - Control a Robotic Arm" (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)
- Antonio Brandi, "Robotics and ROS 2 - Learn by Doing! Manipulators" (<a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>)
