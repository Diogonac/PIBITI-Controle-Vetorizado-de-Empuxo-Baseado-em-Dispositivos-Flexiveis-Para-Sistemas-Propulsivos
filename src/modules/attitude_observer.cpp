#include "attitude_observer.h"
#include "mbed.h"


// Classe do construtor
AttitudeObserver::AttitudeObserver()
    : BNO055(SDA, SCL),
      F2theta_hat(0,0.000661182494267,-0.001195329992332,0.000540248862510,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      F2d_theta_hat(0,-0.031528010174601,0.062613493053578,-0.030842079875511,1.000000000000000,-2.712704387740796,2.452920608962455,-0.739338064889532),
      F2dd_theta_hat(0,0.572614862105168,-1.027685341964908,0.461070754101920,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      theta2theta_hat(0,0.095162581964040,-0.172041288684618,0.077756863052705,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      theta2d_theta_hat(0,-0.002857962996834,-0.000085863861787,0.002417595031668,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      theta2dd_theta_hat(0,-0.585493984460999,1.051454778380353,-0.472033696030868,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      d_theta2theta_hat(0,0.009516258196404,-0.017204128868462,0.007775686305270,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      d_theta2d_theta_hat(0,0.108255440193633,-0.198588790333893,0.091058510222083,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      d_theta2dd_theta_hat(0,-0.151563277010135,0.270149564116311,-0.120351918153069,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      F2phi_hat(0,0.000661182494267,-0.001195329992332,0.000540248862510,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      F2d_phi_hat(0,-0.031528010174601,0.062613493053578,-0.030842079875511,1.000000000000000,-2.712704387740796,2.452920608962455,-0.739338064889532),
      F2dd_phi_hat(0,0.572614862105168,-1.027685341964908,0.461070754101920,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      theta2phi_hat(0,0.095162581964040,-0.172041288684618,0.077756863052705,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      theta2d_phi_hat(0,-0.002857962996834,-0.000085863861787,0.002417595031668,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      theta2dd_phi_hat(0,-0.585493984460999,1.051454778380353,-0.472033696030868,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      d_theta2phi_hat(0,0.009516258196404,-0.017204128868462,0.007775686305270,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      d_theta2d_phi_hat(0,0.108255440193633,-0.198588790333893,0.091058510222083,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534),
      d_theta2dd_phi_hat(0,-0.151563277010135,0.270149564116311,-0.120351918153069,1.000000000000000,-2.712704387740799,2.452920608962460,-0.739338064889534)
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

void AttitudeObserver::estimate(double u_x, double u_y, double theta_in, double Q_in, double phi_in, double P_in){

  estimated_theta[0] = F2theta_hat.update(u_x) + theta2theta_hat.update(theta_in) + d_theta2theta_hat.update(Q_in);
  estimated_theta[1] = F2d_theta_hat.update(u_x) + theta2d_theta_hat.update(theta_in) + d_theta2d_theta_hat.update(Q_in);
  estimated_theta[2] = F2dd_theta_hat.update(u_x) + theta2dd_theta_hat.update(theta_in) + d_theta2dd_theta_hat.update(Q_in);

  estimated_phi[0] = F2phi_hat.update(u_y) + theta2phi_hat.update(phi_in) + d_theta2phi_hat.update(P_in);
  estimated_phi[1] = F2d_phi_hat.update(u_y) + theta2d_phi_hat.update(phi_in) + d_theta2d_phi_hat.update(P_in);
  estimated_phi[2] = F2dd_phi_hat.update(u_y) + theta2dd_phi_hat.update(phi_in) + d_theta2dd_phi_hat.update(P_in);

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
  BNO055.set_orientation(WINDOWS);          // Sentido de rotação ANDROID = Regra da mão direita
  BNO055.set_mapping(3); // Ajuste do eixo de coordenadas P3

  //=================== Calibração do BNO055 =====================
  status_selftest = BNO055.getSystemStatus(BNO055_SELFTEST_RESULT_ADDR); // 15 = todos os sensores então OK
  printf("SelfTest Status: %d \r\n", status_selftest); // Realiza um selftest no BNO055
  wait_ms(25);
  BNO055.setmode(OPERATION_MODE_IMUPLUS); // Configura o mode de fusão entre acelerômetro e giroscópio taxa de atualização máxima: 100Hz
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

  // pitch: Rotação entorno de Y --> Theta
  // Roll: Rotação entorno de X --> Phi
  // Yaw: Rotação entorno de Z --> Psi

  Phi = BNO055.euler.roll;
  Theta = BNO055.euler.pitch;
  Psi = BNO055.euler.yaw;

  P = -BNO055.gyro.y;
  if (abs(BNO055.gyro.x) < 5e-3){
    Q = 0.0;
  } else {
    Q = BNO055.gyro.x;
  }
//   Q = BNO055.gyro.x;
  R = BNO055.gyro.z;
}

