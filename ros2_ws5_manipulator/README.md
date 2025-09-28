## Basic manipulator using ros2 (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)

| term | description | configuration | interrupt handling | MoveIt compatibility |
| - | - | - | - | - |
| forward_command_controller | Sends direct commands (position, velocity, or effort) to a joint or set of joints without trajectory interpolation | simple configuration parameters | immediate overwrite | does work with MoveIt |
| joint_trajectory_controller | Executes full joint trajectories over time. It interpolates between trajectory points, manages timing, and handles smooth motion for multiple joints simultaneously | full PID specification | smooth blending | preferred by MoveIt |

### basic urdf 

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description my_robot.rviz.launch.xacro.py
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

Note also that ``Kinametic Solver: kdl_kinematics_plugin`` for this gripper design is in principle not going to work due to its linear motion design. It should be corrected to ``Kinametic Solver: None``, otherwise it will see the following error (plan/execute still work, however):

    [move_group-6] [ERROR] [1758556501.970869191] [move_group.moveit.moveit.kinematics.kdl_kinematics_plugin]: Group 'gripper' is not a chain
    [move_group-6] [ERROR] [1758556501.971280137] [move_group.moveit.moveit.ros.kinematics_plugin_loader]: Kinematics solver of type 'kdl_kinematics_plugin/KDLKinematicsPlugin' could not be initialized for group 'gripper'
    [move_group-6] [ERROR] [1758556501.971634545] [move_group.moveit.moveit.ros.robot_model_loader]: Kinematics solver could not be instantiated for joint group gripper.

### moveit2 setup assistance with an arm and gripper now with mesh

Mesh obtained from <a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>, lecture 44. Checking the basic rviz model:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description arduinobot.rviz.launch.xacro.py

Launch the MoveIt assistance:

    sudo apt update
    sudo apt install
    colcon build
    source install/setup.bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    # Create New Moveit Configuration Package (or edit if the configuration files already exist)
    # Browse => src/my_robot_description/urdf/arduinobot.urdf.xacro => Load Files
    # ...
    # both arm and gripper group contain the crawler base
    # end effector parent link on the crawler base
    
Fix the following file:

    # src/arduinobot_moveit_config/config/joint_limits.yaml => max_velocity: 10.0, has_acceleration_limits: true, max_acceleration: 1.0 (need to be float)
    # src/arduinobot_moveit_config/config/moveit_controllers.yaml => add in gripper_controller/arm_controller: 
    ## action_ns: follow_joint_trajectory
    ## default: true

Launch the demo:

    colcon build
    source install/setup.bash
    ros2 launch arduinobot_moveit_config demo.launch.py
    # lots of errors, but plan/execute still work
 
### local launch file for moveit2
    
    cp src/ma_robot_moveit_config/config/ros2_controllers.yaml src/my_robot_bringup/config/ma_robot_controllers.yaml
    cp src/ma_robot_moveit_config/config/ma_robot.ros2_control.xacro src/my_robot_description/urdf/
    # adding the following line in ma_robot.ros2_control.xacro:
    ## <xacro:include filename="ma_robot.ros2_control.xacro" />
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup ma_robot.launch.py
    # ignore: [move_group-3] [ERROR] [1758361830.007872451] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
    # Add => MotionPlanning
    # Context => Planning Library => ompl

### command file for moveit2

With just arm:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_moveit_config demo.launch.py
    ros2 run my_robot_commander my_robot_moveit_namedTarget
    ros2 run my_robot_commander my_robot_moveit_jointTarget

With end-effector:

    ros2 launch my_robot_bringup ma_robot.launch.py
    ros2 run my_robot_commander my_robot_moveit_jointTarget
    ros2 run my_robot_commander ma_robot_moveit_poseTarget
    # RobotModel => Links => tool link => Show Trail
    ros2 run my_robot_commander ma_robot_moveit_poseTarget

Using the commander with end-effector (existing interface see <a href="https://github.com/ros2/example_interfaces/tree/rolling/msg">github</a>; custom interface see <a href="https://roboticsbackend.com/ros2-create-custom-message/">webpage</a> or action from ``ros2_ws2_vis_sim`` though in python):

    ros2 launch my_robot_bringup ma_robot.launch.with_commander.py
    # or "ros2 run my_robot_commander ma_robot_commander" after "ros2 launch my_robot_bringup ma_robot.launch.py"
    ros2 topic list
    ros2 topic info /gripper_set_open
    ros2 topic pub -1 /gripper_set_open example_interfaces/msg/Bool "{data: false}"
    ros2 topic pub -1 /gripper_set_open example_interfaces/msg/Bool "{data: true}"
    ros2 topic info /arm_set_name
    ros2 topic pub -1 /arm_set_named example_interfaces/msg/String "{data: "arm_pose1"}"
    ros2 topic info /arm_set_joint
    ros2 topic pub -1 /arm_set_joint example_interfaces/msg/Float32MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
    # try to see error message with: ros2 topic pub -1 /arm_set_joint example_interfaces/msg/Float32MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
    ros2 topic info /arm_set_pose
ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/ArmPoseTarget "{x: 0.0, y: -0.4, z: 0.2, roll: 3.14, pitch: 0.0, yaw: 0.0, use_cartesian_path: false}"
ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/ArmPoseTarget "{x: 0.1, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, use_cartesian_path: true}"
ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/ArmPoseTarget "{x: 0.0, y: 0.1, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, use_cartesian_path: true}"
ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/ArmPoseTarget "{x: 0.0, y: 0.0, z: 0.1, roll: 0.0, pitch: 0.0, yaw: 0.0, use_cartesian_path: true}"

    
## References:
- Edouard Renard, "ROS 2 Moveit 2 - Control a Robotic Arm" (<a href="https://www.udemy.com/course/ros2-moveit2/">Udemy</a>)
- Antonio Brandi, "Robotics and ROS 2 - Learn by Doing! Manipulators" (<a href="https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/">Udemy</a>)
