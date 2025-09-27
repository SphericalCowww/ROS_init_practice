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
   
    // move to pose1 
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    arm.setNamedTarget("pose1");
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success1 == true) {
        arm.execute(plan1);
    }
    // move to pose2
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    arm.setNamedTarget("pose2");
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success2 == true) {
        arm.execute(plan2);
    }
    //////////

    rclcpp::shutdown();
    spinner.join();
    return 0;
}




