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
    //write(0.0);
    largura_pulso(1500);
}

void Servo::write(double percent) {
    double offset = _range_max * 2.0 * (percent - 0.5);
    _pwm.pulsewidth(0.0015 + clamp(offset, _range_min, _range_max));
    _p = clamp(percent, 0.0, 1.0);
}

void Servo::largura_pulso( int largura){

    _pwm.pulsewidth_us(largura);

}

void Servo::position(double degrees) {
    //double offset = _range_max * (degrees / _degrees);
    double offset = 10.8333 * degrees + 550; // 
    
    _pwm.pulsewidth_us(clamp(offset, _range_min, _range_max) + 5.0);
}

void Servo::calibrate(double range_max, double range_min, double degrees) {
    _range_max = range_max;
    _range_min = range_min;
    _degrees = degrees;
   _pwm.period_ms(10);
   
}

double Servo::read() {
    return _p;
}

Servo& Servo::operator= (double percent) { 
    write(percent);
    return *this;
}

Servo& Servo::operator= (Servo& rhs) {
    write(rhs.read());
    return *this;
}

Servo::operator double() {
    return read();
}
