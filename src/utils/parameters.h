#ifndef parameters_h
#define parameters_h

// Tempo de amostragem
const double dt = 0.001; // 50 Hz

// LPF phi constants
const float wc = 100.0;
const float alpha = (wc * dt) / (1 + wc * dt);

// Constantest físicas
const double pi = 3.14159265359f;
const double g = 9.807f;       // m/s^2

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
const double t1 = 0.7275;
const double t2 = 29.2; 
const double p1 = 0.6772;
const double p2 = 27.64; 

// const double T1 = 1.296;
// const double T2 = -34.38; 
// const double P1 = 1.586;
// const double P2 = -50.18; 

const double T1 = 1.292;
const double T2 = -34.26; 
const double P1 = 1.648;
const double P2 = -55.6;

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


// Constantes dos servos
//const double tempo_servos = 0.15;
const double constante_velocidade = 5.0;
const double consteante_phi = 1.0;
const double consteante_theta = 1.0;
const double offset_servo2 = 90.0; //94
const double offset_servo1 = 90.0; //83


// Filtro Kalman
static const double ruido_cov_gx = abs(-0.00876); // Covariancia medida do ruido
static double estima_cov_gx = 0.0005; // Covariancia esperada

static const double ruido_cov_gy = abs(-0.00194); // Covariancia medida do ruido
static double estima_cov_gy = 0.0001; // Covariancia esperada

const double ruido_cov_gz = 0.010474; // Covariancia medida do ruido
static double estima_cov_gz = 0.0001; // Covariancia esperada


static const double ruido_cov_ax = abs(0.001275); // Covariancia medida do ruido
static double estima_cov_ax = 0.0001; // Covariancia esperada

static const double ruido_cov_ay = abs(-0.00088); // Covariancia medida do ruido
static double estima_cov_ay = 0.0001; // Covariancia esperada

static const double ruido_cov_az = abs(0.002108); // Covariancia medida do ruido
static double estima_cov_az = 0.0001; // Covariancia esperada


static const double H = 1.00; //measurement map scalar




#endif