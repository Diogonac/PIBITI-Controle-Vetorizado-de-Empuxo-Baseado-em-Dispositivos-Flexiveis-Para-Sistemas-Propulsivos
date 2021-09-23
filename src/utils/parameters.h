#ifndef parameters_h
#define parameters_h


// Constantest físicas
const double pi = 3.142f;
const double g = 9.807f;       // m/s^2

// Constantes do sistema mecânico
const double m = 0.600f;    // kg
const double I_xx = 0.001221f; // kg.m^2
const double I_yy = 0.001103f; // kg.m^2
const double I_zz = 29.0e-6; // kg.m^2
const double l = 0.0425f;    // m

// Constantes dos controladores
const double KP = 250.0;//537.50f;
const double KD = 27.0;//32.0f;

const double e_max = 15.0;
const double e_min = -15.0;

const double ce_max = 7.5;
const double ce_min = -7.5;


// Constantes dos servos
//const double tempo_servos = 0.15;
const double constante_velocidade = 5.0;
const double consteante_phi = 1.0;
const double consteante_theta = 1.0;
const double offset_servo2 = 94.0;
const double offset_servo1 = 83.0;


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