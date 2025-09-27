#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    arm.setStartStateToCurrentState();    

    // pose goal, require endofactor definition
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    tf2::Quaternion quaternionObj;
    geometry_msgs::msg::PoseStamped target_pose;
    
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = -0.4
    target_pose.pose.position.z = 0.8
    quaternionObj.setRPY(3.14, 0.0, 0.0);   //Euler's angle: (roll, pitch, yaw)
    quaternionObj = quaternionObj.normalize();
    target_pose.pose.orientation.x = quaternionObj.getX();
    target_pose.pose.orientation.y = quaternionObj.getY();
    target_pose.pose.orientation.z = quaternionObj.getZ();
    target_pose.pose.orientation.w = quaternionObj.getW();
    arm.setPoseTarget(target_pose);
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success2 == true) {
        arm.execute(plan2);
    }
    
    // Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double CartesianConstraintStepsize = 0.01;    //meter
    std::vector<geometry_msgs::msg::Pose> waypoints;
 
    geometry_msgs::msg::Pose poseObj1 = arm.getCurrentPose().pose;
    poseObj1.position.z += -0.2;
    waypoints.push_back(poseObj1);

    geometry_msgs::msg::Pose poseObj2 = poseObj1;    
    poseObj2.position.x += 0.2;
    waypoints.push_back(poseObj2);

    geometry_msgs::msg::Pose poseObj3 = poseObj2;
    poseObj3.position.z += 0.2;
    poseObj3.position.x += -0.2;
    waypoints.push_back(poseObj3);    

    double fraction = arm.computeCartesianPath(waypoints, CartesianConstraintStepsize, trajectory);
    if (fraction == 1) {
        arm.execute(trajectory);
    }
    
 
    //////////

    rclcpp::shutdown();
    spinner.join();
    return 0;
}




