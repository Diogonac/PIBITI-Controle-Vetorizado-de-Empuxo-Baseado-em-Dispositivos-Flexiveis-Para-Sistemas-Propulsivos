#ifndef parameters_h
#define parameters_h
#include <cmath>

// Iterruption time
const float dt = 0.01;
const float dt_range = 0.05;
const float dt_wave = 10.0;


// Math constants
const double pi = 3.14159265359;
const double g = 9.8065;                        // m/s^2

// Mecanical constants
const double m = 0.5;                           // kg
const double I_xx = 0.00732;                    // kg.m^2
const double I_yy = 0.00679;                    // kg.m^2
const double I_zz = 0.01346;                    // kg.m^2
const double l = 0.02779;                       // m
const double Fmax = 7.0;                        // N
const double AngleSat = 12.0 * pi / 180.0;      // Rad
const double Fsat = Fmax * sin(AngleSat);       // N

// IMU pre-filter
const double wA = 0.3203;
const double wB = 0.3203;
const double wC = 0.3594;


// Attitude controller gains
// const float Ts_att = 2.0;
// const float OS_att = 0.5;
// const float zeta_att = -log(OS_att/100)/sqrt(pow(pi, 2) + pow((log(OS_att/100)), 2));
// const float wn_att = 4 / (zeta_att * Ts_att);
// const float K1 = pow(wn_att, 2);
// const float K2 = 2 * zeta_att * wn_att;
// double K_theta[3] = {2.0736, 1.5915, 0.4389};

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



// Vertical estimator constants
const double wc = 8; //aprox. 500Hz antes estava em 8
const double zeta = sqrt(2)/2;
const double l1 = wc*wc;
const double l2 = 2 * zeta * wc; 

// Vertical controller gains
const float Ts_vert = 5.0;
const float OS_vert = 1.0;
const float zeta_vert = -log(OS_vert/100)/sqrt(pow(pi, 2) + pow((log(OS_vert/100)), 2));
const float wn_vert = 4 / (zeta_vert * Ts_vert);
const float kp_vert = pow(wn_vert, 2);
const float kd_vert = 2 * zeta_vert * wn_vert;

// Valve curve fit constants
const double coef1 = -0.004041;
const double coef2 = 0.03335;
const double coef3 = -0.1218;
const double coef4 = 0.9867;
const double coef5 = -0.6387;
const double coef6 = 4.466;

#endif