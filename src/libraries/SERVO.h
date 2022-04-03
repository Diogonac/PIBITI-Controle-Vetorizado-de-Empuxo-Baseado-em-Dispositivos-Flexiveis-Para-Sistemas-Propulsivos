#ifndef SERVO_H
#define SERVO_H

#include "mbed.h"

class Servo {

public:
    // Create a servo object connected to the specified PwmOut pin
    Servo(PinName pin);

    void pulse_width( int width);
    
    void position(double degrees);
       
    void calibrate(double range_max = 2500, double range_min = 550, double degrees = 180.0);
        


protected:
    PwmOut _pwm;
    double _range_max;
    double _range_min;
    double _degrees;
};

#endif
