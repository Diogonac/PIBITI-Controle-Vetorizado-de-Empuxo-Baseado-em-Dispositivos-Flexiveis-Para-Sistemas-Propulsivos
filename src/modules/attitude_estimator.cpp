#include "attitude_estimator.h"

// Classe do estimador de atitude
AttitudeEstimator::AttitudeEstimator():BNO055(SDA, SCL)
{
    Phi = 0.0;
    Theta = 0.0;
    Psi = 0.0;

    P = 0.0;
    Q = 0.0;
    R = 0.0;

    // Condições iniciais dos offset dos ângulos de euler
    offset_phi = 0.0;
    offset_theta = 0.0;
    offset_psi = 0.0;

    // Condições iniciais dos offset das velocidades angulares
    offset_gx = 0.0;
    offset_gy = 0.0;
    offset_gz = 0.0;

    verifica_imu = false;

}


// Inicializa a IMU
void AttitudeEstimator::init()
{

    //============= Configurações iniciais no BNO055 ================
    BNO055.reset(); //Reseta o BNO055
    status_BNO055 = BNO055.getSystemStatus(BNO055_SYS_ERR_ADDR); //Verifica o status do BNO055
    status_check_BNO055 = BNO055.check(); //Verifica se tem comunicação com o BNO055
    printf("Status BNO055: %d\r\n", status_BNO055); //Indica 0 para operação nominal do BNO055
    printf("Verifica comunicacao BNO055: %d\r\n", status_check_BNO055); //Indica 1 para operação nominal do BNO055
    //BNO055.SetExternalCrystal(true); //Indica a existencia de um cristal externo
    wait_ms(675);
    BNO055.setpowermode(POWER_MODE_NORMAL); //Define o modo de alimentação do BNO055
    wait_ms(25); //Aguarda a troca de modo de alimentação
    BNO055.setmode(OPERATION_MODE_CONFIG); //Configura o modo padrão para iniciar a calibração
    wait_ms(25); //Aguarda o BNO055 trocar de modo de operação

//============= Unidade das variáveis do BNO055 ================
    BNO055.set_accel_units(MPERSPERS); // m/s2
    BNO055.set_anglerate_units(RAD_PER_SEC); // rad/s
    BNO055.set_angle_units(RADIANS); // Radiano
    BNO055.set_temp_units(CENTIGRADE); // °C
    BNO055.set_orientation(ANDROID); // Sentido de rotação ANDROID = Regra da mão direita
    BNO055.set_mapping(1); // Ajuste do eixo de coordenadas / orientação P4 for calib and P1 for others


    //=================== Calibração do BNO055 =====================
    status_selftest = BNO055.getSystemStatus(BNO055_SELFTEST_RESULT_ADDR); //15 = todos os sensores então OK
    printf("SelfTest Status: %d \r\n", status_selftest); //Realiza um selftest no BNO055
    wait_ms(25);
    BNO055.setmode(OPERATION_MODE_IMUPLUS); //Configura o mode de fusão entre acelerômetro e giroscópio taxa de atualização máxima: 100Hz
    wait_ms(25); //Aguarda o BNO055 trocar de modo de operação OPERATION_MODE_NDOF  OPERATION_MODE_IMUPLUS


    if(status_selftest == 15 && status_check_BNO055 == 1 && status_BNO055 == 0) {

        verifica_imu = true;

    } else {

        verifica_imu = false;
    }

    wait(1);
    
    // Define os offset dos ângulos de euler
    BNO055.get_angles();

    offset_phi = BNO055.euler.pitch;
    offset_theta = BNO055.euler.roll;
    offset_psi = BNO055.euler.yaw;

    // Define os offset das velocidades angulares
    BNO055.get_gyro();

    offset_gx = BNO055.gyro.x;
    offset_gy = BNO055.gyro.y;
    offset_gz = BNO055.gyro.z;

    printf("Offset dos angulos (rad) \r\n");
    printf("Phi= %f, Theta= %f, Psi= %f \r\n", offset_phi, offset_theta, offset_psi);
    printf("\r\n");

    printf("Offset das velocidades angular (rad/s) \r\n");
    printf("GX= %f, GY= %f, GZ= %f \r\n", offset_gx, offset_gy, offset_gz);
    printf("\r\n");

}

// Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
void AttitudeEstimator::estimate(void)
{

    BNO055.get_angles();

    Phi = -BNO055.euler.pitch;
    Theta = -BNO055.euler.roll;
    Psi =  BNO055.euler.yaw;

    BNO055.get_gyro();

    // Verificar os sentidos dessas rotações!!!!
    // Provavelmente devo adicionar um - em P e Q
    P = BNO055.gyro.x - offset_gx; 
    Q = BNO055.gyro.y - offset_gy; 
    R = BNO055.gyro.z - offset_gz; 

}