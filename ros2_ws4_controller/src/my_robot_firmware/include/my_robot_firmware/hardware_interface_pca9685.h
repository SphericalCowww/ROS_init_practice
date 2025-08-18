#ifndef HARDWARE_INTERFACE_PCA9685_HPP
#define HARDWARE_INTERFACE_PCA9685_HPP

#include "hardware_interface/system_interface.hpp"
#include "pca9685/PCA9685.h"

namespace my_robot_hardware {
    
    class HardwareInterfacePCA9685: public hardware_interface::SystemInterface
    {
        public:
            // lifecycle node override
            hardware_interface::CallbackRetrun on_configure (const rclcpp_lifecycle:State & previous state) override;
            hardware_interface::CallbackRetrun on_activate  (const rclcpp_lifecycle:State & previous state) override;
            hardware_interface::CallbackRetrun on_deactivate(const rclcpp_lifecycle:State & previous state) override;
            // SystemInterface override
            hardware_interface::CallbackRetrun on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::return_type    read (const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type    write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        pricate:
            std::shared_ptr<PCA9685> pwm_controller_;
            int pwm_freq      = 50;
            int i2c_address   = 0x40;
            int servo_channel = 0;
    }    

}

#endif
