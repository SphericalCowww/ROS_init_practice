#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
class Commander
{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node) {
            node_ = node;
            I_arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            I_arm_->setMaxVelocityScalingFactor(1.0);
            I_arm_->setMaxAccelerationScalingFactor(1.0);
            I_gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            open_gripper_sub = node_->create_subscription<Bool>("open_gripper", 10, );

            double CartesianConstraintStepsize_ = 0.01;    //meter
        }
        void armNamedTarget(const std::string &name) {
            I_arm_->setStartStateToCurrentState();
            I_arm_->setNamedTarget(name);
            planAndExecute(I_arm_);
        }
        void armJointTarget(const std::vector<double> &joints) {
            I_arm_->setStartStateToCurrentState();
            I_arm_->setJointValueTarget(joints);
            planAndExecute(I_arm_);
        }
        void armPoseTarget(double x, double y, double z, double roll, double pitch, double yaw,
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

            I_arm_->setStartStateToCurrentState();
            if (use_cartesian_path == false) {
            
                I_arm_setPoseTarget(target_pose);
                planAndExecute(I_arm_);
            } else {
                moveit_msgs::msg::RobotTrajectory trajectory;
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                double fraction = I_arm_.computeCartesianPath(waypoints, CartesianConstraintStepsize, trajectory);
                if (fraction == 1) {
                    I_arm_.execute(trajectory);
                }
            }
        }
        void gripperNameTarget(const std::string &name) {
            I_gripper_->setStartStateToCurrentState();
            I_gripper_->setNamedTarget(name);
            planAndExecute(I_gripper_);
        }
        void gripperOpen()  { gripperNameTarget("gripper_open"); }
        void gripperClose() { gripperNameTarget("gripper_close"); }
    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
            MoveGroupInterface::Plan plan;
            interface->plan(plan);
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success == true) {
                interface->execute(plan);
            }
        }
        void OpenGripperCallback(const Bool &msg) {
            if (msg.data) {
                openGripper();
            } else {
                closeGripper();
            }
            
        }
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> I_arm_;
        std::shared_ptr<MoveGroupInterface> I_gripper_;
        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander")l
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




