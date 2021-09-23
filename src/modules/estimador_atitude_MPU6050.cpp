#include "estimador_atitude_MPU6050.h"

// Classe do estimador de atitude
EstimadorAtitudeMPU::EstimadorAtitudeMPU() : MPU6050(MPU_SDA, MPU_SCL)
{
    Phi_MPU = 0.0;
    Theta_MPU = 0.0;
    Psi_MPU = 0.0;
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

    wait_ms(10); //Aguarda o deslocamento

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
}


