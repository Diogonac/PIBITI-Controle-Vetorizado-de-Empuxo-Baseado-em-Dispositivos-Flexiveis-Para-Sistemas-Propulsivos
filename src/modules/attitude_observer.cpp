#include "attitude_observer.h"
#include "mbed.h"


// Classe do construtor
AttitudeObserver::AttitudeObserver() : BNO055(SDA, SCL) {

  // Observer variables init
  for (int i = 0; i < 4; i++) {
    theta_log[i] = 0.0;
  }

  for (int i = 0; i < 4; i++) {
    d_theta_log[i] = 0.0;
  }

  for (int i = 0; i < 4; i++) {
    u_log[i] = 0.0;
  }

  for (int i = 0; i < 3; i++) {
    estimated[i] = 0.0;
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        state_hat[i][j] = 0.0;
    }
  }

  // IMU variables init
  Phi = 0.0;
  Theta = 0.0;
  Psi = 0.0;

  P = 0.0;
  Q = 0.0;
  R = 0.0;

  verifica_imu = false;

}

void AttitudeObserver::estimate(double u) {

  readIMU();

  calculate(u_log, const1, const2, const3, const4);
  calculate(theta_log, const1, const5, const6, const7);
  calculate(d_theta_log, const1, const8, const9, const10);

  // Shifting positions
  state_hat[0][2] = state_hat[0][1];
  state_hat[0][1] = state_hat[0][0];
  state_hat[0][0] = estimated[0];       // This is the theta estimated

  state_hat[1][2] = state_hat[1][1];
  state_hat[1][1] = state_hat[1][0];
  state_hat[1][0] = estimated[1];       // This is the d_theta estimated

  state_hat[2][2] = state_hat[2][1];
  state_hat[2][1] = state_hat[2][0];
  state_hat[2][0] = estimated[2];       // This is the dd_theta estimated

  // Cleaning estimations
  estimated[0] = 0.0;
  estimated[1] = 0.0;
  estimated[2] = 0.0;

  // States logging
  u_log[3] = u_log[2];
  u_log[2] = u_log[1];
  u_log[1] = u_log[0];
  u_log[0] = u;

  theta_log[3] = theta_log[2];
  theta_log[2] = theta_log[1];
  theta_log[1] = theta_log[0];
  theta_log[0] = Theta;

  d_theta_log[3] = d_theta_log[2];
  d_theta_log[2] = d_theta_log[1];
  d_theta_log[1] = d_theta_log[0];
  d_theta_log[0] = Q;
}

void AttitudeObserver::calculate(double input_past[4], double num[3], double den1[4], double den2[4], double den3[4]) {

  estimated[0] += num[0] * state_hat[0][0] + num[1] * state_hat[0][1] +
                  num[2] * state_hat[0][2] + den1[0] * input_past[0] +
                  den1[1] * input_past[1] + den1[2] * input_past[2] +
                  den1[3] * input_past[3];
  estimated[1] += num[0] * state_hat[1][0] + num[1] * state_hat[1][1] +
                  num[2] * state_hat[1][2] + den2[0] * input_past[0] +
                  den2[1] * input_past[1] + den2[2] * input_past[2] +
                  den2[3] * input_past[3];
  estimated[2] += num[0] * state_hat[2][0] + num[1] * state_hat[2][1] +
                  num[2] * state_hat[2][2] + den3[0] * input_past[0] +
                  den3[1] * input_past[1] + den3[2] * input_past[2] +
                  den3[3] * input_past[3];
}

// Inicializa a IMU
void AttitudeObserver::initIMU() {

  //============= Configurações iniciais no BNO055 ================
  BNO055.reset(); // Reseta o BNO055
  status_BNO055 = BNO055.getSystemStatus(BNO055_SYS_ERR_ADDR); // Verifica o status do BNO055
  status_check_BNO055 = BNO055.check(); // Verifica se tem comunicação com o BNO055
  printf("Status BNO055: %d\r\n", status_BNO055); // Indica 0 para operação nominal do BNO055
  printf("Verifica comunicacao BNO055: %d\r\n", status_check_BNO055); // Indica 1 para operação nominal do BNO055
  wait_ms(675);
  BNO055.setpowermode(POWER_MODE_NORMAL); // Define o modo de alimentação do BNO055
  wait_ms(25);            
  BNO055.setmode(OPERATION_MODE_CONFIG); // Configura o modo padrão para iniciar
  wait_ms(25); 

  //============= Unidade das variáveis do BNO055 ================
  BNO055.set_accel_units(MPERSPERS);       // m/s2
  BNO055.set_anglerate_units(RAD_PER_SEC); // rad/s
  BNO055.set_angle_units(RADIANS);         // Radiano
  BNO055.set_temp_units(CENTIGRADE);       // °C
  BNO055.set_orientation(ANDROID);          // Sentido de rotação ANDROID = Regra da mão direita
  BNO055.set_mapping(1); // Ajuste do eixo de coordenadas P1

  //=================== Calibração do BNO055 =====================
  status_selftest = BNO055.getSystemStatus(BNO055_SELFTEST_RESULT_ADDR); // 15 = todos os sensores então OK
  printf("SelfTest Status: %d \r\n", status_selftest); // Realiza um selftest no BNO055
  wait_ms(25);
  BNO055.setmode( OPERATION_MODE_NDOF); // Configura o mode de fusão entre acelerômetro e giroscópio taxa de atualização máxima: 100Hz
  wait_ms(25);              

  if (status_selftest == 15 && status_check_BNO055 == 1 && status_BNO055 == 0) {

    verifica_imu = true;

  } else {

    verifica_imu = false;
  }

  wait(1);
}

void AttitudeObserver::readIMU(void) {

  BNO055.get_angles();
  BNO055.get_gyro();

  Phi = -BNO055.euler.pitch;
  Theta = -BNO055.euler.roll;
  Psi = BNO055.euler.yaw;

  P = BNO055.gyro.x;
  Q = BNO055.gyro.y;
  R = BNO055.gyro.z;

}

// /* Controle das forças de empuxo (N) dado meu ângulo de referência (rad),
// ângulo atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q ->
// theta_ponto */ void AttitudeController::control(double phi_r, double theta_r,
// double phi, double theta, double p, double q, double p_r, double q_r)
// {

//     f_x = controller(theta_r, theta, q, K1, K2, Ki);// * (I_yy/l);

//     f_y = controller(phi_r, phi, p, K1, K2, Ki);// * (I_yy/l);

//     //printf("%f %f\r\n", f_x, f_y);

// }

// // Controlador siso
// double AttitudeController::controller(double angulo_r, double angulo, double
// v_angular, double K1, double K2, double Ki)
// {

//     // Valores atuais
//     pos_erro = angulo_r - angulo;
//     vel_erro = 0 - v_angular;

//     // u_control = 1.5199 * pos_erro + 0.5598 * vel_erro + 2.4412 *
//     pos_erro_int;

//     proportional = K1 * pos_erro;
//     derivative = K2 * vel_erro;

//     if (abs(u_control) <= 1.8) {
//       integrator = integrator + Ki * dt * 0.5 * (pos_erro + pos_erro_past);
//       integrator_past = integrator;
//     } else {
//       integrator = integrator_past;
//     }

//     pos_erro_past = pos_erro;
//     vel_erro_past = vel_erro;

//     u_control = proportional + derivative + integrator;

//     return u_control * K;
//     // return pos_erro*5.4063 + vel_erro*4.0;

// }