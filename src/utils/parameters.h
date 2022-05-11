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
const double m = 0.50282;    // kg
const double I_xx = 0.006080156; // kg.m^2
const double I_yy = 0.006948864; // kg.m^2
const double I_zz = 0.012574241; // kg.m^2
const double l = 0.0393;    // m

// Constantes das curvas de calibração dos ângulos
const double T1 = 1.744;//1.55;
const double T2 = -55.82;//-39.48;
const double P1 = 1.201;//1.233;
const double P2 = -23.83;//-18.58;

// Constantes dos controladores
const double K1_att = 33.7895;
const double K2_att = 10.0;
// const double KP = 10.966045224442436;
// const double KD = 26.666666666666668;

const double e_max = 15.0;
const double e_min = -15.0;

const double ce_max = 7.5;
const double ce_min = -7.5;

// Servos
const double offset_servo1 = 90.0;//93.0558;//95.1885;
const double offset_servo2 = 90.0;//96.6845;//90.0;//94.8701;
const double phi_safe = (P1 * offset_servo1) + P2;
const double theta_safe = (T1 * offset_servo2) + T2;
const double time_displacement = 4.0; // In ms

// Vertical estimator constants
const double wc = 8; //aprox. 500Hz antes estava em 8
const double zeta = sqrt(2)/2;
const double l1 = wc*wc;
const double l2 = 2 * zeta * wc; 

// Vertical controller gains
const float Ts_vert = 1.0;
const float OS_vert = 2.0;
const float zeta_vert = -log(OS_vert/100)/sqrt(pow(pi, 2) + pow((log(OS_vert/100)), 2));
const float wn_vert = 4 / (zeta_vert * Ts_vert);
const float kp_vert =  10.8127;//5.8487; //pow(wn_vert, 2);
const float kd_vert = 4.0;//2.6667;//2 * zeta_vert * wn_vert;

// Valve curve fit constants
const double coef1 = -0.004041;
const double coef2 = 0.03335;
const double coef3 = -0.1218;
const double coef4 = 0.9867;
const double coef5 = -0.6387;
const double coef6 = 4.466;

#endif