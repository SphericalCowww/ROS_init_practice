#include <unistd.h>
#include <math.h>
#include "pca9685/PCA9685.h"

// Correct define for the internal oscillator frequency
#define CLOCK_FREQ 25000000.0

PCA9685::PCA9685(int bus, int address) {
    i2c = new I2C(bus, address);
    reset();
    // Do NOT set a default frequency here. Let the user's main code do it.
}

PCA9685::~PCA9685() {
    delete i2c;
}

void PCA9685::reset() {
    i2c->write_byte(MODE1, 0x00); // Reset registers to default
    usleep(10000); // Wait 10ms for reset
    i2c->write_byte(MODE2, 0x04); // Configure for totem-pole output
}

/**
 * BUG FIX #1: This function implements the reliable wake-up sequence
 * as required by the datasheet.
 */
void PCA9685::setPWMFreq(int freq) {
    // Calculate prescale value from datasheet formula
    uint8_t prescale_val = (uint8_t)floor((CLOCK_FREQ / 4096.0 / (double)freq) - 1.0);

    // Get the current mode register value
    uint8_t oldmode = i2c->read_byte(MODE1);
    
    // Put the chip to sleep (required to change prescaler)
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // set sleep bit
    i2c->write_byte(MODE1, newmode);
    
    // Set the prescaler
    i2c->write_byte(PRE_SCALE, prescale_val);
    
    // Restore the original mode to wake the chip up
    i2c->write_byte(MODE1, oldmode);
    
    // REQUIRED: Wait at least 500us for the oscillator to stabilize
    usleep(500);
    
    // Set the RESTART bit to re-enable PWM outputs
    i2c->write_byte(MODE1, oldmode | 0x80);
}

/**
 * BUG FIX #2: The channel address calculation is now correct for 0-indexed channels.
 * It no longer uses `(led - 1)`.
 */
void PCA9685::setPWM(uint8_t channel, int on_value, int off_value) {
    // Ensure values are within the 12-bit range
    on_value &= 0xFFF;
    off_value &= 0xFFF;

    i2c->write_byte(LED0_ON_L + (LED_MULTIPLYER * channel), on_value & 0xFF);
    i2c->write_byte(LED0_ON_H + (LED_MULTIPLYER * channel), on_value >> 8);
    i2c->write_byte(LED0_OFF_L + (LED_MULTIPLYER * channel), off_value & 0xFF);
    i2c->write_byte(LED0_OFF_H + (LED_MULTIPLYER * channel), off_value >> 8);
}

// These helper functions are not strictly needed for your test but are good to have.
void PCA9685::setPWM(uint8_t led, int value) {
    setPWM(led, 0, value);
}

int PCA9685::getPWM(uint8_t led) {
    int ledval = 0;
    ledval = i2c->read_byte(LED0_OFF_H + LED_MULTIPLYER * led);
    ledval = ledval & 0xf;
    ledval <<= 8;
    ledval += i2c->read_byte(LED0_OFF_L + LED_MULTIPLYER * led);
    return ledval;
}
