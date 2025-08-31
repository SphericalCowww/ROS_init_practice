#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_controller/me_controller.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace me_controller {

    MeController::MeController(): controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn MeController::on_init()
    {   
        node_ = std::make_shared<rclcpp::Node>("MeController_node");
        RCLCPP_INFO(node_->get_logger(), "MeController_node::on_init()");
        
        joint_names_    = auto_declare<std::vector<std::string>>("joints", {});
        interface_name_ = auto_declare<std::string>             ("interface_name", "position");
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn MeController::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(node_->get_logger(), "MeController_node::on_configure()");
        (void) previous_state;
        // this is a lambda function to aboid repeated parameters in FloatArray
        auto callback = [this] (const FloatArray::SharedPtr msg) -> void
        {
            if (msg->data.size() == joint_names_.size()) {
                appCommand_.clear();
                for (auto cmd: msg->data) {
                    appCommand_.push_back(cmd);
                }
            }
        };
        // for available functions see: 
        // https://control.ros.org/rolling/doc/api/classcontroller__interface_1_1ControllerInterfaceBase.html
        command_subscriber_ = get_node()->create_subscription<FloatArray>(get_name()+"/joints_command", 10, callback);
        return CallbackReturn::SUCCESS;
    }
    controller_interface::InterfaceConfiguration MeController::command_interface_configuration() const
    {
        RCLCPP_INFO(node_->get_logger(), "MeController_node::command_interface_configuration()");
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (auto joint_name: joint_names_) {
            config.names.push_back(joint_name+"/"+interface_name_);
        }
        return config;
    }
    controller_interface::InterfaceConfiguration MeController::state_interface_configuration() const
    {
        RCLCPP_INFO(node_->get_logger(), "MeController_node::state_interface_configuration()");
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (auto joint_name: joint_names_) {
            config.names.push_back(joint_name+"/"+interface_name_);
        }
        return config;
    }
    controller_interface::CallbackReturn MeController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(node_->get_logger(), "MeController_node::on_activate()");
        (void) previous_state;
        appCommand_.clear();
        for (int joint_idx = 0; joint_idx < (int)joint_names_.size(); joint_idx++) {
            appCommand_.push_back(state_interfaces_[joint_idx].get_optional().value());
        }
        return CallbackReturn::SUCCESS; 
    }
    controller_interface::return_type MeController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        RCLCPP_INFO(node_->get_logger(), "MeController_node::update()");
        (void) time;
        (void) period;      // can define update rate
        for (int joint_idx = 0; joint_idx < (int)joint_names_.size(); joint_idx++) {
            double state_val = state_interfaces_[joint_idx].get_optional().value();
            double cmd_val = appCommand_[joint_idx];
            double new_val = cmd_val + state_val;                   // NOTE: ADDING NEW VALUES TO OLD
            (void) command_interfaces_[joint_idx].set_value(new_val);
        }
        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(me_controller::MeController, controller_interface::ControllerInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









