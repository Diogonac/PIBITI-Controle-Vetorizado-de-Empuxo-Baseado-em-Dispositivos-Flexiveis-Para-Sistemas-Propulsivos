#include "calibration.h"

// Classe do construtor
Calibration::Calibration()
    : servo1(SERVO1), servo2(SERVO2), BNO055(SDA, SCL), valve(SDA, SCL) {

  calibration = false;
  calibration_phi = false;
  calibration_theta = false;
  init_dac = false;

  sum_phi = 0.0;
  sum_theta = 0.0;

  estimated_phi = 0.0;
  estimated_theta = 0.0;

  mean_phi = 0.0;
  mean_theta = 0.0;

  Phi = 0.0;
  Theta = 0.0;
  Psi = 0.0;

  P = 0.0;
  Q = 0.0;
  R = 0.0;

  // Condições iniciais dos offset das velocidades angulares
  offset_gx = 0.0;
  offset_gy = 0.0;
  offset_gz = 0.0;
}

void Calibration::config_dac() {

  if (valve.open()) {
    printf("Device detected!\n");
    valve.wakeup();
    init_dac = true;
  } else {
    printf("Device not detected!\n");
  }
}

void Calibration::calib_dac() {
  for (int i = 19; i >=0; i--) {
    valve.write(DAC[i]);
    printf("DAC: %f [V]\r\n", DAC[i]);
    wait(20);
  }
}

void Calibration::check_calibration(bool calib_phi, bool calib_theta) {

  if (calib_phi == true && calib_theta == true) {

    calibration = true;
  }

  else {
    calibration = false;
  }
}

void Calibration::calibra_servo_phi(void) {

  printf("\r\n");
  printf("Calibração servo 1 ângulo phi iniciada");
  printf("\r\n");
  servo1.position(offset_servo1);
  servo2.position(offset_servo2);
  wait(1);
  estimate();
  wait(1);

  for (int i = 0; i < 12; i++) {
    servo1.position(angle_calib_table[i] + offset_servo1);
    servo2.position(offset_servo2);
    wait(2);
    sum_phi = 0.0;

    for (int y = 0; y < 500; y++) {
      estimate();
      estimated_phi = (Phi * 180.0 / pi);
      sum_phi += estimated_phi;
      wait_ms(2);

      // printf("%f %f\r\n", estimated_phi, lista_angulos_1[i] + offset_servo1);
    }

    mean_phi = sum_phi / 500;
    phi_data_calib[i] = mean_phi + offset_servo1;
    printf("%f %f\r\n", mean_phi + offset_servo1,
           angle_calib_table[i] + offset_servo1);
    wait(2);
  }

  printf("Calibração servo 1 ângulo phi finalizada\r\n");
  printf("Resultados\r\n");

  printf("Angulos_REF= [");
  for (int k = 0; k < 12; k++) {
    printf(", %f", angle_calib_table[k]);
  }
  printf(" ];");

  printf("\r\n");
  printf("Angulos_Phi= [");
  for (int k = 0; k < 12; k++) {
    printf(", %f", phi_data_calib[k]);
  }
  printf(" ];");

  calibration_phi = true;
}

void Calibration::calibra_servo_theta(void) {

  printf("\r\n");
  printf("Calibração servo 2 ângulo theta iniciada");
  printf("\r\n");
  servo1.position(offset_servo1);
  servo2.position(offset_servo2);
  wait(1);
  estimate();
  wait(1);

  for (int i = 0; i < 12; i++) {
    servo1.position(offset_servo1);
    servo2.position(angle_calib_table[i] + offset_servo2);
    wait(2);
    sum_theta = 0.0;

    for (int y = 0; y < 500; y++) {
      estimate();
      estimated_theta = (Theta * 180.0 / pi);
      sum_theta += estimated_theta;
      wait_ms(2);

      // printf("%f %f\r\n", estimated_theta, lista_angulos_1[i] +
      // offset_servo2);
    }

    mean_theta = sum_theta / 500;
    theta_data_calib[i] = mean_theta + offset_servo2;
    printf("%f %f\r\n", mean_theta + offset_servo2,
           angle_calib_table[i] + offset_servo2);
    wait(2);
  }


  printf("Calibração servo 1 ângulo theta finalizada\r\n");
  printf("Resultados\r\n");

  printf("Angulos_REF= [");
  for (int k = 0; k < 12; k++) {
    printf(", %f", angle_calib_table[k]);
  }
  printf(" ];");

  printf("\r\n");
  printf("Angulos_Theta= [");
  for (int k = 0; k < 12; k++) {
    printf(", %f", theta_data_calib[k]);
  }
  printf(" ];");

  calibration_theta = true;
}

void Calibration::angle_calib(double angle, double c1, double c2) {

  angle_calibrated = (c1 * angle) + c2;
}

void Calibration::phi_test_calib(void) {

  printf("\r\n");
  printf("Verificação da calibração servo 1 ângulo phi iniciada");
  printf("\r\n");
  angle_calib(offset_servo1, P1, P1);
  servo1.position(angle_calibrated);
  angle_calib(offset_servo2, T1, T2);
  servo2.position(angle_calibrated);
  wait(1);
  estimate();
  wait(1);

  for (int i = 0; i < 11; i++) {
    angle_calib(angle_calib_table[i] + offset_servo1, P1, P2);
    servo1.position(angle_calibrated);
    angle_calib(offset_servo2, T1, T2);
    servo2.position(angle_calibrated);
    wait(2);
    sum_phi = 0.0;

    for (int y = 0; y < 500; y++) {
      estimate();
      estimated_phi = (Phi * 180.0 / pi);
      sum_phi += estimated_phi;
      wait_ms(2);

    //   printf("%f %f\r\n", estimated_phi + offset_servo1, angle_calib_table[i] + offset_servo1);
    }

    mean_phi = sum_phi / 500;
    phi_data_calib[i] = mean_phi + offset_servo1;
    angle_calib(angle_calib_table[i] + offset_servo1, P1, P2);
    printf("%f %f\r\n", mean_phi + offset_servo1,angle_calib_table[i] + offset_servo1);
    wait(2);
  }

  angle_calib(offset_servo1, P1, P2);
  servo1.position(angle_calibrated);
  angle_calib(offset_servo2, T1, T2);
  servo2.position(angle_calibrated);
  wait(1);

  for (int i = 0; i < 11; i++) {
    angle_calib(offset_servo2, P1, P2);
    servo1.position(angle_calibrated);
    angle_calib(angle_calib_table[i] + offset_servo2, T1, T2);
    servo2.position(angle_calibrated);
    wait(2);
    sum_theta = 0.0;

    for (int y = 0; y < 500; y++) {
      estimate();
      estimated_theta = (Theta * 180.0 / pi);
      sum_theta += estimated_theta;
      wait_ms(2);

    //   printf("%f %f\r\n", estimated_theta + offset_servo2, angle_calib_table[i] + offset_servo2);
    }

    mean_theta = sum_theta / 500;
    theta_data_calib[i] = mean_theta + offset_servo2;
    angle_calib(angle_calib_table[i] + offset_servo2, T1, T2);
    printf("%f %f\r\n", mean_theta + offset_servo2, angle_calib_table[i] + offset_servo1);
    wait(2);
  }

  angle_calib(offset_servo1, P1, P2);
  servo1.position(angle_calibrated);
  angle_calib(offset_servo2, T1, T2);
  servo2.position(angle_calibrated);
  wait(1);

  printf("Verificação da calibração servo 1 ângulo phi finalizada\r\n");
}

// Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
void Calibration::estimate(void) {

  BNO055.get_angles();

  // Pitch e Roll estão trocados conforme a convensão utilizada

  Phi = -BNO055.euler
            .pitch; // - offset_phi; // Ângulo formado pela rotação de (x) (phi)
  Theta = -BNO055.euler.roll; // - offset_theta; // Ângulo formado pela rotação
                             // de (y) (theta)
  Psi = BNO055.euler.yaw; // - offset_psi; //(z) (psi)

  BNO055.get_gyro();

  P = BNO055.gyro.x -
      offset_gx; // kalman_gx(BNO055.gyro.x, ruido_cov_gx, estima_cov_gx);
  Q = BNO055.gyro.y -
      offset_gy; // kalman_gy(BNO055.gyro.y, ruido_cov_gy, estima_cov_gy);
  R = BNO055.gyro.z -
      offset_gz; // kalman_gz(BNO055.gyro.z, ruido_cov_z, estima_cov_gz);
}

// Inicializa a IMU
void Calibration::config_calib_imu() {

  //============= Configurações iniciais no BNO055 ================
  BNO055.reset(); // Reseta o BNO055
  status_BNO055 = BNO055.getSystemStatus(
      BNO055_SYS_ERR_ADDR); // Verifica o status do BNO055
  status_check_BNO055 =
      BNO055.check(); // Verifica se tem comunicação com o BNO055
  printf("Status BNO055: %d\r\n",
         status_BNO055); // Indica 0 para operação nominal do BNO055
  printf("Verifica comunicacao BNO055: %d\r\n",
         status_check_BNO055);     // Indica 1 para operação nominal do BNO055
  BNO055.SetExternalCrystal(true); // Indica a existencia de um cristal externo
  wait_ms(675);
  BNO055.setpowermode(
      POWER_MODE_NORMAL); // Define o modo de alimentação do BNO055
  wait_ms(25);            // Aguarda a troca de modo de alimentação
  BNO055.setmode(OPERATION_MODE_CONFIG); // Configura o modo padrão para iniciar
                                         // a calibração
  wait_ms(25); // Aguarda o BNO055 trocar de modo de operação

  //============= Unidade das variáveis do BNO055 ================
  BNO055.set_accel_units(MPERSPERS);       // m/s2
  BNO055.set_anglerate_units(RAD_PER_SEC); // rad/s
  BNO055.set_angle_units(RADIANS);         // GRAUS
  BNO055.set_temp_units(CENTIGRADE);       // °C
  BNO055.set_orientation(ANDROID);          // Sentido de rotação ANDROID = Regra da mão direita
  BNO055.set_mapping(2); // Ajuste do eixo de coordenadas / orientação P4 for
                         // calib and P1 for others

  //=================== Calibração do BNO055 =====================
  status_selftest = BNO055.getSystemStatus(
      BNO055_SELFTEST_RESULT_ADDR); // 15 = todos os sensores então OK
  printf("SelfTest Status: %d \r\n",
         status_selftest); // Realiza um selftest no BNO055
  wait_ms(25);
  BNO055.setmode(
      OPERATION_MODE_IMUPLUS); // Configura o mode de fusão entre acelerômetro e
                               // giroscópio taxa de atualização máxima: 100Hz
  wait_ms(25); // Aguarda o BNO055 trocar de modo de operação
               // OPERATION_MODE_NDOF  OPERATION_MODE_IMUPLUS
}