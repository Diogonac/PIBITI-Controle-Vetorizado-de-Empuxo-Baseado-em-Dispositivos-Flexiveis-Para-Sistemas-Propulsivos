#include "estimador_atitude_MPU6050.h"

// Classe do estimador de atitude
EstimadorAtitudeMPU::EstimadorAtitudeMPU() : MPU6050(MPU_SDA, MPU_SCL), servo1(SERVO1), servo2(SERVO2)
{
    Phi_MPU = 0.0;
    Theta_MPU = 0.0;
    Psi_MPU = 0.0;

    pos = 90.0;
    
    of_1 = 0.0;
    of_2 = 0.0;

//    P = 0.0;
//    Q = 0.0;
//    R = 0.0;
//
//    // Condições iniciais dos offset dos ângulos de euler
//    offset_phi = 0.0;
//    offset_theta = 0.0;
//    offset_psi = 0.0;
//
//    // Condições iniciais dos offset das velocidades angulares
//    offset_gx = 0.0;
//    offset_gy = 0.0;
//    offset_gz = 0.0;

}


// Inicializa a IMU
void EstimadorAtitudeMPU::config_MPU()
{

    printf("Iniciando MPU6050\r\n");

    if(MPU6050.testConnection() == 1) {

        printf("MPU6050 - Status: OK\r\n");

    } else {

        printf("MPU6050 - Status: Sem resposta \r\n");

    }

    MPU6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    MPU6050.setGyroRange(MPU6050_GYRO_RANGE_250);
    MPU6050.setBW(MPU6050_BW_20);

    //=================== Calibração dos servos =====================
    servo1.calibrate(0.0019, 0.001, 180.0); //Define as configurações do servo 1: pulsewidth_MÁX / pulsewidth_MíN / Ângulo de varredura total
    servo2.calibrate(0.0019, 0.001, 180.0); //Define as configurações do servo 2: pulsewidth_MÁX / pulsewidth_MíN / Ângulo de varredura total

//=================== Verificação dos servos ====================
    servo1.position(75.0 + of_1); //Define a posição inicial do servo 1
    servo2.position(75.0 + of_2); //Define a posição inicial do servo 2
    wait_ms(10); //Aguarda o deslocamento

    wait(1);

}

// Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
void EstimadorAtitudeMPU::estima_MPU()
{

    MPU6050.getAccelero(acc_MPU);

// Pitch e Roll estão trocados conforme a convensão utilizada

    ax = acc_MPU[0];
    ay = acc_MPU[1];
    az = acc_MPU[2];

    Phi_MPU = atan2((ay),(az)) * 180 / pi;
    Theta_MPU = atan2((ax),(az)) * 180 / pi;


//    BNO055.get_gyro();
//
//    P = BNO055.gyro.x - offset_gx; //kalman_gx(BNO055.gyro.x, ruido_cov_gx, estima_cov_gx);
//    Q = BNO055.gyro.y - offset_gy; //kalman_gy(BNO055.gyro.y, ruido_cov_gy, estima_cov_gy);
//    R = BNO055.gyro.z - offset_gz; //kalman_gz(BNO055.gyro.z, ruido_cov_z, estima_cov_gz);


    //printf("AX= %f, AY= %f,AZ= %f\r\n", ax, ay, az);

}


void EstimadorAtitudeMPU::calibra_angulo(int tempo, double incremento)
{

    for (pos = 70; pos <= 110; pos += incremento) {
        //Varre a abertura de 90º até 105º com um incremento de 1º
        servo1.position((pos + of_1) * consteante_phi); //Imprime o Ângulo no servo 1
        //servo2.position(pos + of_2); //Imprime o Ângulo no servo 2

        estima_MPU();
        printf("%f,%f,%f,%f\r\n", ax, ay, az, pos);

        wait_ms(tempo); //Aguarda o deslocamento
    }

    for (pos = 110; pos >= 70; pos -= incremento) {
        //Varre a abertura de 105º até 75º com um incremento de 1º
        servo1.position((pos + of_1) * consteante_phi); //Imprime o Ângulo no servo 1
        //servo2.position(pos + of_2); //Imprime o Ângulo no servo 2

        estima_MPU();
        printf("%f,%f,%f,%f\r\n", ax, ay, az, pos);

        wait_ms(tempo); //Aguarda o deslocamento
    }

}