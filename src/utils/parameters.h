#ifndef parameters_h
#define parameters_h
#include <cmath>

// Iterruption time
const float dt = 0.01;
const float dt_range = 0.05;

// Constantest físicas
const double pi = 3.14159265359f;
const double g = 9.8065;       // m/s^2

// Constantes do sistema mecânico
const double m = 0.5;           // kg
const double I_xx = 0.00732;    // kg.m^2
const double I_yy = 0.00679;    // kg.m^2
const double I_zz = 0.01346;    // kg.m^2
const double l = 0.02779;       // m


// Attitude controller gains
const float Ts_att = 1.0;
const float OS_att = 0.5;
const float zeta_att = -log(OS_att/100)/sqrt(pow(pi, 2) + pow((log(OS_att/100)), 2));
const float wn_att = 4 / (zeta_att * Ts_att);
const float K1 = pow(wn_att, 2);
const float K2 = 2 * zeta_att * wn_att;
// const double K1_att_phi = -K1_att_theta;
// const double K2_att_phi = -K2_att_theta;

const double e_max = 15.0;
const double e_min = -15.0;

const double ce_max = 7.5;
const double ce_min = -7.5;

// Servos
const double offset_servo1 = 90.0;
const double offset_servo2 = 90.0;
const double T1 = 1.511;
const double T2 = -44.01;
const double P1 = 1.286;
const double P2 = -34.34;
const double phi_safe = (P1 * offset_servo1) + P2;
const double theta_safe = (T1 * offset_servo2) + T2;

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