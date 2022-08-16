#ifndef parameters_h
#define parameters_h
#include <cmath>

// Iterruption time
const float dt = 0.01;
const float dt_range = 0.05;

// Math constants
const double pi = 3.14159265359;
const double g = 9.8065;                        // m/s^2

// Mecanical constants
const double m = 0.5;                           // kg
const double I_xx = 0.00720;                    // kg.m^2
const double I_yy = 0.00749;                    // kg.m^2
const double I_zz = 0.01418;                    // kg.m^2
const double l = 0.04029;                       // m
const double Fmax = 7.0;                        // N
const double AngleSat = 12.0 * pi / 180.0;      // Rad
const double Fsat = Fmax * sin(AngleSat);       // N

// Servos
const double offset_servo1 = 90.0;
const double offset_servo2 = 90.0;
const double T1 = 1.514; 
const double T2 = 2.846;
const double P1 = 1.26; 
const double P2 = 0.9991;
const double phi_safe = (P1 * 0) + P2;
const double theta_safe = (T1 * 0) + T2;
const double phi_max = 106.119;
const double phi_min = 75.879;
const double theta_max = 111.014;
const double theta_min = 74.678;

// Valve curve fit constants
const double coef1 = -0.004041;
const double coef2 = 0.03335;
const double coef3 = -0.1218;
const double coef4 = 0.9867;
const double coef5 = -0.6387;
const double coef6 = 4.466;

#endif