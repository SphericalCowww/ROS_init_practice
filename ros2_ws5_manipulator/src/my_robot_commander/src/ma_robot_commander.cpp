#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using namespace std::placeholders;
class ma_robot_commander_class
{
    public:
        ma_robot_commander_class(std::shared_ptr<rclcpp::Node> node) {
            node_ = node;
            arm_interface_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            arm_interface_->setMaxVelocityScalingFactor(1.0);
            arm_interface_->setMaxAccelerationScalingFactor(1.0);
            gripper_interface_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            gripper_subscriber_ = node_->create_subscription<Bool>("gripper_set_open", 10, 
                std::bind(&ma_robot_commander_class::gripperCallback, this, _1));
        }
        void armSetNamedTarget(const std::string &name) {
            arm_interface_->setStartStateToCurrentState();
            arm_interface_->setNamedTarget(name);
            planAndExecute(arm_interface_);
        }
        void armSetJointTarget(const std::vector<double> &joints) {
            arm_interface_->setStartStateToCurrentState();
            arm_interface_->setJointValueTarget(joints);
            planAndExecute(arm_interface_);
        }
        void armSetPoseTarget(double x, double y, double z, double roll, double pitch, double yaw,
                              bool use_cartesian_path=false) {
            tf2::Quaternion quaternionObj;
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";    

            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            quaternionObj.setRPY(roll, pitch, yaw);   //Euler's angle
            quaternionObj = quaternionObj.normalize();
            target_pose.pose.orientation.x = quaternionObj.getX();
            target_pose.pose.orientation.y = quaternionObj.getY();
            target_pose.pose.orientation.z = quaternionObj.getZ();
            target_pose.pose.orientation.w = quaternionObj.getW();

            arm_interface_->setStartStateToCurrentState();
            if (use_cartesian_path == false) {
                arm_interface_->setPoseTarget(target_pose);
                planAndExecute(arm_interface_);
            } else {
                moveit_msgs::msg::RobotTrajectory trajectory;
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                double fraction = arm_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_, trajectory);
                if (fraction == 1) {
                    arm_interface_->execute(trajectory);
                }
            }
        }
        void gripperSetNameTarget(const std::string &name) {
            gripper_interface_->setStartStateToCurrentState();
            gripper_interface_->setNamedTarget(name);
            planAndExecute(gripper_interface_);
        }
        void gripperOpen()  { gripperSetNameTarget("gripper_open"); }
        void gripperClose() { gripperSetNameTarget("gripper_close"); }
    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
            MoveGroupInterface::Plan plan;
            interface->plan(plan);
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success == true) {
                interface->execute(plan);
            }
        }
        void gripperCallback(const Bool &msg) {
            if (msg.data) {
                gripperOpen();
            } else {
                gripperClose();
            }
        }
        
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_interface_;
        std::shared_ptr<MoveGroupInterface> gripper_interface_;
        rclcpp::Subscription<Bool>::SharedPtr gripper_subscriber_;
        double cartesianConstraintStepsize_ = 0.01;    //meter
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ma_robot_commander");
    auto commander = ma_robot_commander_class(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




