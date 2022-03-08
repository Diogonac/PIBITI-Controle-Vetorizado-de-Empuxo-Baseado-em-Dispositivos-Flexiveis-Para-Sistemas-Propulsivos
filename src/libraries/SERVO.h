#ifndef MBED_SERVO_H
#define MBED_SERVO_H

#include "mbed.h"

class Servo {

public:
    /** Create a servo object connected to the specified PwmOut pin
     *
     * @param pin PwmOut pin to connect to 
     */
    Servo(PinName pin);
    
    /** Set the servo position, normalised to it's full range
     *
     * @param percent A normalised number 0.0-1.0 to represent the full range.
     */
    void write(double percent);

    void largura_pulso( int largura);
    
    /**  Read the servo motors current position
     *
     * @param returns A normalised number 0.0-1.0  representing the full range.
     */
    double read();
    
    /** Set the servo position
     *
     * @param degrees Servo position in degrees
     */
    void position(double degrees);
    
    /**  Allows calibration of the range and angles for a particular servo
     *
     * @param range Pulsewidth range from center (1.5ms) to maximum/minimum position in seconds
     * @param degrees Angle from centre to maximum/minimum position in degrees
     */
    //void calibrate(float range = 0.0015, float degrees = 60.0);     
    void calibrate(double range_max = 2500, double range_min = 550, double degrees = 180.0);
        
    /**  Shorthand for the write and read functions */
    Servo& operator= (double percent);
    Servo& operator= (Servo& rhs);
    operator double();

protected:
    PwmOut _pwm;
    double _range_max;
    double _range_min;
    double _degrees;
    double _p;
};

#endif
