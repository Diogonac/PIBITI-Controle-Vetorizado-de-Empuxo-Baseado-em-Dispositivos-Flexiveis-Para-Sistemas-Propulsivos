#include "SERVO.h"
#include "mbed.h"

static double clamp(double value, double min, double max) {
    if(value < min) {
        return min;
    } else if(value > max) {
        return max;
    } else {
        return value;
    }
}

Servo::Servo(PinName pin) : _pwm(pin) {
    calibrate(2500, 550, 180.0);
    //pulse_width(1500); // Initial position
}

void Servo::pulse_width( int width){

    _pwm.pulsewidth_us(width);

}

void Servo::position(double degrees) {

    double offset = 10.8333 * degrees + 550; // Curve fit
    _pwm.pulsewidth_us(clamp(offset, _range_min, _range_max) + 5.0);
}

void Servo::calibrate(double range_max, double range_min, double degrees) {
    _range_max = range_max;
    _range_min = range_min;
    _degrees = degrees;
   _pwm.period_ms(10);
   
}