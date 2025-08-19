#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_pca9685.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_hardware {
    hardware_interface::CallbackRetrun HardwareInterfacePCA968::on_init
        (const hardware_interface::HardwareInfo & info) 
    {
        RCLCPP_INFO(node->get_logger(), "HardwareInterfacePCA968:on_init()");
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackRetrun::SUCCESS) {
            return hardware_interface::CallbackRetrun::ERROR;
        }
        
        info_ = info;        
        pwm_controller_ = std::shared_ptr<PCA9685>(i2c_bus_, i2c_address_);
        
        return hardware_interface::CallbackRetrun::SUCCESS;     
    }
    hardware_interface::return_type HardwareInterfacePCA968::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
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
    hardware_interface::return_type HardwareInterfacePCA968::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        (void) time;
        (void) period; 
        
        // see: /src/my_robot_description/urdf/mobile_base.ros2_control.xacro
        int delay_ms = 100;
        for (int ticks = min_ticks_; ticks <= max_ticks_; ++ticks) {
            pwm_controller.setPWM(right_servo_channel, 0, ticks);
            delay_time = delay_ms/set_command("base_right_wheel_joint/velocity")
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        }
        for (int ticks = min_ticks_; ticks <= max_ticks_; ++ticks) {
            pwm_controller.setPWM(left_servo_channel, 0, ticks);
            delay_time = delay_ms/set_command("base_left_wheel_joint/velocity")
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        }    
        return hardware_interface::return_type::OK;
    }   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    hardware_interface::CallbackRetrun HardwareInterfacePCA968::on_configure 
        (const rclcpp_lifecycle:State & previous_state) 
    {
        RCLCPP_INFO(node->get_logger(), "HardwareInterfacePCA968:on_configure()");
        (void) previous_state;
        pwm_controller_.setPWMFreq(pwm_freq_); 
    }
    hardware_interface::CallbackRetrun HardwareInterfacePCA968::on_activate  
        (const rclcpp_lifecycle:State & previous state) 
    {
        RCLCPP_INFO(node->get_logger(), "HardwareInterfacePCA968:on_activate()");
        (void) previous_state;
        pwm_controller_.setPWM(right_servo_channel_, 0, min_ticks_);
        pwm_controller_.setPWM(left_servo_channel_,  0, min_ticks_);

        set_state("base_right_wheel_joint/position", 0.0);
        set_state("base_left_wheel_joint/position",  0.0);
        set_state("base_right_wheel_joint/velocity", 0.0);
        set_state("base_left_wheel_joint/velocity",  0.0);
        return hardware_interface::CallbackRetrun::SUCCESS;
    }
    hardware_interface::CallbackRetrun HardwareInterfacePCA968::on_deactivate
        (const rclcpp_lifecycle:State & previous state) 
    {
        RCLCPP_INFO(node->get_logger(), "HardwareInterfacePCA968:on_deactivate()");
        (void) previous_state;
        pwm_controller_.setPWM(right_servo_channel_, 0, 0);
        pwm_controller_.setPWM(left_servo_channel_,  0, 0);
        return hardware_interface::CallbackRetrun::SUCCESS;
    }
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



