#ifndef parameters_h
#define parameters_h
#include <cmath>

// Iterruption time
const float dt = 0.01;
const float dt_range = 0.05;


// Constantest físicas
const double pi = 3.14159265359f;
const double g = 9.807;       // m/s^2

// Constantes do sistema mecânico
const double m = 0.600f;    // kg
const double I_xx = 0.001221f; // kg.m^2
const double I_yy = 0.001103f; // kg.m^2
const double I_zz = 29.0e-6; // kg.m^2
const double l = 0.0425f;    // m

// Constantes das curvas de calibração dos ângulos
/* 
(Ângulo no servo) = [(Ângulo desejado no ponto central) - (t2 ou p2)] / (t1 ou p1)
*/

const double T1 = 1;//0.78;//0.7871;
const double T2 = 0;//21.09;//19.31; 
const double P1 = 1.317;//1.026;
const double P2 = -28.73;//2.457 ;

const double T1_f = 0.9813;
const double T2_f = 1.449; 
const double P1_f = 0.9998;
const double P2_f = -0.03386; 


// Constantes dos controladores
const double KP = 300.0;//215.0;//488.0;//250.0;//537.50f;
const double KD = 35.0;//22.5;//45.0;//27.0;//32.0f;
// const double KP = 10.966045224442436;
// const double KD = 26.666666666666668;

const double e_max = 15.0;
const double e_min = -15.0;

const double ce_max = 7.5;
const double ce_min = -7.5;


// Servos
const double offset_servo2 = 90.0; //94
const double offset_servo1 = 90.0; //83
const double phi_safe = (P1 * offset_servo1) + P2;
const double theta_safe = (T1 * offset_servo2) + T2;
const double time_displacement = 4.0; // In ms

// Vertical estimator constants
const double wc = 10; //aprox. 500Hz antes estava em 8
const double zeta = sqrt(2)/2;
const double l1 = wc*wc;
const double l2 = 2 * zeta * wc; 

// Valve curve fit constants
const double p1 = -0.004041;
const double p2 = 0.03335;
const double p3 = -0.1218;
const double p4 = 0.9867;
const double p5 = -0.6387;
const double p6 = 4.466;


#endif