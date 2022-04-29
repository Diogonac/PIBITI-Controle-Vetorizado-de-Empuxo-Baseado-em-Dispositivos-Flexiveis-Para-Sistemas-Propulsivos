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
const double m = 0.5;    // kg
const double I_xx = 0.001221f; // kg.m^2
const double I_yy = 0.001103f; // kg.m^2
const double I_zz = 29.0e-6; // kg.m^2
const double l = 0.0425f;    // m

// Constantes das curvas de calibração dos ângulos
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

// Vertical controller gains
const float Ts_vert = 1.0;//0.8;
const float OS_vert = 2.0;//1.0;
const float zeta_vert = -log(OS_vert/100)/sqrt(pow(pi, 2) + pow((log(OS_vert/100)), 2));
const float wn_vert = 4 / (zeta_vert * Ts_vert);
const float kd_vert = 4.0;//2.6667;//2 * zeta_vert * wn_vert;
const float kp_vert =  10.8127;//5.8487; //pow(wn_vert, 2);

// Valve curve fit constants
const double coef1 = -0.004041;
const double coef2 = 0.03335;
const double coef3 = -0.1218;
const double coef4 = 0.9867;
const double coef5 = -0.6387;
const double coef6 = 4.466;

// Feed-Foward constants
const double F_coulomb = 1.2;// 1.0
const double viscous = 1.0;  // 0.8
const double K1 = 0.07;

// o modelo tem que admitir uma faixa de velocidade baixa
// devo filtrar melhor o sinal do sinal 


// const double f_stiction = 1.2;
// const double f_coulomb = f_stiction/1.01;
// const double f_max = 7.0;
// const double K1 = 0.001;
// const double K2 = 0.1;
// const double K3 = 0.2;

// const double K2 = 5.0;
// const double K3 = 5.0;


#endif