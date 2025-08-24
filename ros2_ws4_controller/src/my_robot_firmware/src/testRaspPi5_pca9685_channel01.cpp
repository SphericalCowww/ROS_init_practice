#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "pca9685/PCA9685.h"

// --- Servo Configuration ---

// Most servos operate at 50Hz (a 20ms period).
#define PWM_FREQUENCY 50

// You can fine-tune these values for your specific servo.
// A common range for many servos is 600µs to 2400µs.
// Standard is often 1000µs (0 deg) to 2000µs (180 deg).
#define MIN_PULSE_WIDTH_US 600  // Minimum pulse width in microseconds for 0 degrees.
#define MAX_PULSE_WIDTH_US 2400 // Maximum pulse width in microseconds for 180 degrees.

// Default I2C address for the PCA9685
#define PCA9685_I2C_ADDRESS 0x40

/**
 * @brief Calculates the PCA9685 tick value from a pulse width in microseconds.
 * @param pulse_us The desired pulse width in microseconds.
 * @param pwm_freq The frequency the PCA9685 is running at.
 * @return The 12-bit tick value to send to the PCA9685.
 */
int us_to_ticks(int pulse_us, int pwm_freq) {
    double pulse_s = pulse_us / 1000000.0;     // Convert us to seconds
    double period_s = 1.0 / pwm_freq;          // Get period in seconds
    double ticks_per_s = 4096 / period_s;      // 12-bit resolution
    return static_cast<int>(pulse_s * ticks_per_s);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pca9685_servo_sweep_node");
    RCLCPP_INFO(node->get_logger(), "Starting PCA9685 servo sweep test.");

    int i2c_bus = 1;
    
    try {
        PCA9685 pwm_controller(i2c_bus, PCA9685_I2C_ADDRESS);
        pwm_controller.setPWMFreq(PWM_FREQUENCY);

        // Convert our microsecond pulse widths into the 12-bit tick values the PCA9685 needs
        int min_ticks = us_to_ticks(MIN_PULSE_WIDTH_US, PWM_FREQUENCY);
        int max_ticks = us_to_ticks(MAX_PULSE_WIDTH_US, PWM_FREQUENCY);

        RCLCPP_INFO(node->get_logger(), "PCA9685 Initialized at %d Hz.",   PWM_FREQUENCY);
        RCLCPP_INFO(node->get_logger(), "Servo Range (us): %d to %d",      MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
        RCLCPP_INFO(node->get_logger(), "Calculated Tick Range: %d to %d", min_ticks, max_ticks);
        
        int servo_channel0 = 0;
        int servo_channel1 = 1;
        pwm_controller.setPWM(servo_channel0, 0, max_ticks);
        pwm_controller.setPWM(servo_channel1, 0, max_ticks);

        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        pwm_controller.setPWM(servo_channel0, 0);
        pwm_controller.setPWM(servo_channel1, 0);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "An exception occurred: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

