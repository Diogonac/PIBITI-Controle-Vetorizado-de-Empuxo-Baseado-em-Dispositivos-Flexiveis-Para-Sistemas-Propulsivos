#include "attitude_estimator.h"
#include "mbed.h"

//  DirectOne(double b0, double b1, double b2, double b3, double a0, double a1, double a2, double a3);

// Classe do construtor
AttitudeEstimator::AttitudeEstimator()
    : BNO055(SDA, SCL),
      F2theta_hat(-0.0000182524909, 0.00275978533, 0.002778037821, 0.0, 1.0, -1.0, 0.25, 0.0),
      F2d_theta_hat(0.4911280815, 0.4159032417, -0.07522483983, 0.0, 1.0, -1.0, 0.25, 0.0),
      theta2theta_hat(0.4216455903, 0.1222497926, -0.2993957978, 0.0, 1.0, -1.0, 0.25, 0.0),
      theta2d_theta_hat(10.1926796, -0.3525357129, -10.54521532, 0.0, 1.0, -1.0, 0.25, 0.0)
      {

  // Observer variables init
  

  // IMU variables init
  Phi = 0.0;
  Theta = 0.0;
  Psi = 0.0;

  P = 0.0;
  Q = 0.0;
  R = 0.0;

  verifica_imu = false;
}

void AttitudeEstimator::estimate(double u_x, double u_y, double theta_in, double Q_in, double phi_in, double P_in){

  estimated_theta[0] = F2theta_hat.update(u_x) + theta2theta_hat.update(theta_in);// + d_theta2theta_hat.update(Q_in);
  estimated_theta[1] = F2d_theta_hat.update(u_x) + theta2d_theta_hat.update(theta_in);// + d_theta2d_theta_hat.update(Q_in);
  estimated_theta[2] = 0.0;//F2dd_theta_hat.update(u_x) + theta2dd_theta_hat.update(theta_in);// + d_theta2dd_theta_hat.update(Q_in);

//   estimated_phi[0] = F2phi_hat.update(u_y) + theta2phi_hat.update(phi_in);
//   estimated_phi[1] = F2d_phi_hat.update(u_y) + theta2d_phi_hat.update(phi_in);
//   estimated_phi[2] = F2dd_phi_hat.update(u_y) + theta2dd_phi_hat.update(phi_in);

}

// Inicializa a IMU
void AttitudeEstimator::init() {

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
  BNO055.set_orientation(WINDOWS);          // Sentido de rotação ANDROID = Regra da mão direita
  BNO055.set_mapping(3); // Ajuste do eixo de coordenadas P3
  wait_ms(25);
  BNO055.config_range(GYRO_CONFIG, GYRO_RANGE_1000DPS);
  wait_ms(25);
  //=================== Calibração do BNO055 =====================
  status_selftest = BNO055.getSystemStatus(BNO055_SELFTEST_RESULT_ADDR); // 15 = todos os sensores então OK
  printf("SelfTest Status: %d \r\n", status_selftest); // Realiza um selftest no BNO055
  wait_ms(25);
  BNO055.setmode(OPERATION_MODE_NDOF); // Configura o mode de fusão entre acelerômetro e giroscópio taxa de atualização máxima: 100Hz
  wait_ms(25);              

  if (status_selftest == 15 && status_check_BNO055 == 1 && status_BNO055 == 0) {

    verifica_imu = true;

  } else {

    verifica_imu = false;
  }

  wait(1);
}

void AttitudeEstimator::read(void) {

  BNO055.get_angles();
  BNO055.get_gyro();

  // pitch: Rotação entorno de Y --> Theta
  // Roll: Rotação entorno de X --> Phi
  // Yaw: Rotação entorno de Z --> Psi

  Phi = BNO055.euler.roll;
  Theta = BNO055.euler.pitch;
  Psi = BNO055.euler.yaw;

//   P = -BNO055.gyro.y;
//   Q = BNO055.gyro.x;
//   R = BNO055.gyro.z;

  P = -BNO055.gyro.y * 0.1358 + 0.1358 * P + 0.7285 * P;
  Q = BNO055.gyro.x * 0.1358 + 0.1358 * Q + 0.7285 * Q;
  R = BNO055.gyro.z * 0.1358 + 0.1358 * R + 0.7285 * R;
}

