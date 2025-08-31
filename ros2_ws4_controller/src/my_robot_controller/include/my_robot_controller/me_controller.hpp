#ifndef ME_CONTROLLER_HPP
#define ME_CONTROLLER_HPP

#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
using FloatArray = example_interfaces::msg::Float64MultiArray;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace me_controller {
    class MeController: public controller_interface::ControllerInterface
    {
        public:
            MeController();
            // interface override
            controller_interface::InterfaceConfiguration command_interface_configuration() const override; 
            controller_interface::InterfaceConfiguration state_interface_configuration()   const override;
            controller_interface::return_type 
                update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            // lifecycle node override
            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn 
                on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn 
                on_activate(const rclcpp_lifecycle::State & previous_state) override;
        protected:
            std::shared_ptr<rclcpp::Node>               node_;
            rclcpp::Subscription<FloatArray>::SharedPtr command_subscriber_;
            std::vector<std::string>                    joint_names_;
            std::string                                 interface_name_;
            std::vector<double>                         appCommand_;
    };
}



#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






