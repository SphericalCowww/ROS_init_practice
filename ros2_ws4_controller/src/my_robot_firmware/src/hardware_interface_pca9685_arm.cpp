#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_pca9685_arm.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace ma_robot_arm {
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_arm::on_init
        (const hardware_interface::HardwareInfo & info) 
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        node_ = std::make_shared<rclcpp::Node>("HardwareInterfacePCA9685_arm_node");
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_arm::on_init()");
 
        info_ = info;        
        pwm_controller_ = std::make_shared<PCA9685>(i2c_bus_, i2c_address_);
        joint1_servo_channel_ = std::stoi(info_.hardware_parameters["joint1_servo_channel"]);
        joint2_servo_channel_  = std::stoi(info_.hardware_parameters["joint2_servo_channel"]);
        pwm_min_microSec_ = std::stoi(info_.hardware_parameters["pwm_min_microSec"]);
        pwm_max_microSec_ = std::stoi(info_.hardware_parameters["pwm_max_microSec"]);
        microSec_to_ticks(pwm_min_microSec_, pwm_freq_);
        microSec_to_ticks(pwm_max_microSec_, pwm_freq_);

        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::return_type HardwareInterfacePCA9685_arm::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_arm::read()");
        (void) period;
        if (write_first_call == true) {
            start_time = time;
            write_first_call = false;
        }
        rclcpp::Duration lifetime = time - start_time;
    
        // note: feedback not available for pca9685
        double joint1_position = get_command("arm_joint1/position");  
        double joint2_position = get_command("arm_joint2/position"); 
        RCLCPP_INFO(node_->get_logger(), "position (joint1, joint2): (%lf, %lf)", joint1_position, joint2_position);
        // see: /src/my_robot_description/urdf/mobile_base.ros2_control.xacro
        set_state("arm_joint1/position", joint1_position);
        set_state("arm_joint2/position", joint2_position);
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfacePCA9685_arm::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_arm::write()");
        (void) time;
        (void) period; 
        
        // see: /src/my_robot_description/urdf/arm.ros2_control.xacro
        pwm_controller_->setPWM(joint1_servo_channel_, 0, get_command("arm_joint1/position"));
        pwm_controller_->setPWM(joint2_servo_channel_, 0, get_command("arm_joint2/position")); 
        return hardware_interface::return_type::OK;
    }   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_arm::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_arm::on_configure()");
        (void) previous_state;
        pwm_controller_->setPWMFreq(pwm_freq_); 
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_arm::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_arm::on_activate()");
        (void) previous_state;
        pwm_controller_->setPWM(joint1_servo_channel_, 0, min_ticks_);
        pwm_controller_->setPWM(joint2_servo_channel_, 0, min_ticks_);

        set_state("arm_joint1/position", 0.0);
        set_state("arm_joint2/position", 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_arm::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_arm::on_deactivate()");
        (void) previous_state;
        pwm_controller_->setPWM(joint1_servo_channel_, 0, 0);
        pwm_controller_->setPWM(joint2_servo_channel_, 0, 0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ma_robot_arm::HardwareInterfacePCA9685_arm, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









