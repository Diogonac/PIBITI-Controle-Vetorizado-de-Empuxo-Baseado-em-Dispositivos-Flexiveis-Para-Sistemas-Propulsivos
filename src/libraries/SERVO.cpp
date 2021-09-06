/* mbed R/C Servo Library
 *  
 * Copyright (c) 2007-2010 sford, cstyles
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
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
    calibrate();
    write(0.5);
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
    //float offset = _range_max * (degrees / _degrees);
    double offset = 11 * degrees + 550; // 
    
    _pwm.pulsewidth_us(clamp(offset, _range_min, _range_max));
}

void Servo::calibrate(double range_max, double range_min, double degrees) {
    _range_max = range_max;
    _range_min = range_min;
    _degrees = degrees;
   _pwm.period_ms(20);
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
