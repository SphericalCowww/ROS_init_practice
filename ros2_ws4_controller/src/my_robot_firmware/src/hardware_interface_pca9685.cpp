#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_pca9685.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_firmware {
    hardware_interface::CallbackReturn HardwareInterfacePCA9685::on_init
        (const hardware_interface::HardwareInfo & info) 
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        node_ = std::make_shared<rclcpp::Node>("HardwareInterfacePCA9685_node");
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685::on_init()");
 
        info_ = info;        
        pwm_controller_ = std::make_shared<PCA9685>(i2c_bus_, i2c_address_);

        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::return_type HardwareInterfacePCA9685::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685::read()");
        (void) time;
        double right_velocity = 0.0;        //feedback not available for pca9685
        double left_velocity  = 0.0;        //feedback not available for pca9685

        // see: /src/my_robot_description/urdf/mobile_base.ros2_control.xacro
        set_state("base_right_wheel_joint/velocity", right_velocity);
        set_state("base_left_wheel_joint/velocity",  left_velocity);
        set_state("base_right_wheel_joint/position", 
                  get_state("base_right_wheel_joint/position") + right_velocity*period.seconds());
        set_state("base_left_wheel_joint/position",
                  get_state("base_left_wheel_joint/position")  + left_velocity*period.seconds());
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfacePCA9685::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685::write()");
        (void) time;
        (void) period; 
        
        // see: /src/my_robot_description/urdf/mobile_base.ros2_control.xacro
        int delay_ms = 100;
        int delay_time;
        for (int ticks = min_ticks_; ticks <= max_ticks_; ++ticks) {
            pwm_controller_->setPWM(right_servo_channel_, 0, ticks);
            delay_time = delay_ms/get_command("base_right_wheel_joint/velocity");
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        }
        for (int ticks = min_ticks_; ticks <= max_ticks_; ++ticks) {
            pwm_controller_->setPWM(left_servo_channel_, 0, ticks);
            delay_time = delay_ms/get_command("base_left_wheel_joint/velocity");
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        }    
        return hardware_interface::return_type::OK;
    }   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::CallbackReturn HardwareInterfacePCA9685::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685::on_configure()");
        (void) previous_state;
        pwm_controller_->setPWMFreq(pwm_freq_); 
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfacePCA9685::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685::on_activate()");
        (void) previous_state;
        pwm_controller_->setPWM(right_servo_channel_, 0, min_ticks_);
        pwm_controller_->setPWM(left_servo_channel_,  0, min_ticks_);

        set_state("base_right_wheel_joint/position", 0.0);
        set_state("base_left_wheel_joint/position",  0.0);
        set_state("base_right_wheel_joint/velocity", 0.0);
        set_state("base_left_wheel_joint/velocity",  0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfacePCA9685::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685::on_deactivate()");
        (void) previous_state;
        pwm_controller_->setPWM(right_servo_channel_, 0, 0);
        pwm_controller_->setPWM(left_servo_channel_,  0, 0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_firmware::HardwareInterfacePCA9685, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









