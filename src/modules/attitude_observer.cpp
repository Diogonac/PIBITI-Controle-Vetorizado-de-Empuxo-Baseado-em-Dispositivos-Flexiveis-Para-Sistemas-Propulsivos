#include "attitude_observer.h"
#include "mbed.h"


// Classe do construtor
AttitudeObserver::AttitudeObserver()
    : BNO055(SDA, SCL),
      F2theta_hat(0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      F2d_theta_hat(0.000303339223415,0.000332228673264,-0.000245560323717,-0.000274449773566,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      F2dd_theta_hat(0.065746956639935,-0.049327117808113,-0.064779506743389,0.050294567704659,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      theta2theta_hat(0.047619047619048,-0.038462451266583,-0.047178909622841,0.038902589262790,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      theta2d_theta_hat(-0.000241946445562,-0.000264988964187,0.000195861408312,0.000218903926937,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      theta2dd_theta_hat(-0.052440440396918,0.039343810171087,0.051668792533206,-0.040115458034800,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      d_theta2theta_hat(0.004761904761905,-0.003846245126658,-0.004717890962284,0.003890258926279,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      d_theta2d_theta_hat(0.077956707897241,-0.066347356259345,-0.077558145681362,0.066745918475224,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018),
      d_theta2dd_theta_hat(0.152648696479544,-0.145420246477279,-0.153344841345356,0.144724101611467,1.000000000000000,-2.712473381360156,2.452502853345587,-0.739149195993018)
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

void AttitudeObserver::estimate(double u, double theta_in, double Q_in, double phi_in, double P_in){

  estimated[0] = theta_in;//F2theta_hat.update(u) + theta2theta_hat.update(theta_in) + d_theta2theta_hat.update(Q_in);
  estimated[1] = Q_in;//F2d_theta_hat.update(u) + theta2d_theta_hat.update(theta_in) + d_theta2d_theta_hat.update(Q_in);
  estimated[2] = F2dd_theta_hat.update(u) + theta2dd_theta_hat.update(theta_in) + d_theta2dd_theta_hat.update(Q_in);

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

    //   F2theta_hat(0,0.000000000000000,0.000000000000000,0.000000000000000,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   F2d_theta_hat(0,0.000626082742516,0.000018809859375,-0.000529613059860,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   F2dd_theta_hat(0,0.131519782716873,-0.230240193828951,0.100650651763849,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   theta2theta_hat(0,0.095162581964040,-0.172041288684618,0.077756863052705,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   theta2d_theta_hat(0,-0.000499369954448,-0.000015002934886,0.000422424755736,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   theta2dd_theta_hat(0,-0.104901514519542,0.183641917109342,-0.080279982138785,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   d_theta2theta_hat(0,0.009516258196404,-0.017204128868462,0.007775686305270,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   d_theta2d_theta_hat(0,0.155673327480881,-0.288176101111066,0.133297978626430,1.000000000000000,-2.712704387740798,2.452920608962458,-0.739338064889533),
    //   d_theta2dd_theta_hat(0,0.304293170939851,-0.594224430871049,0.288542322763176,1.000000000000000,-2.712704387740795,2.452920608962453,-0.739338064889531)
    //   {