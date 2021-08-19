#include "mbed.h"
#include "imports.h"


//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

//======= Configurações para reiniciar a placa ======
DigitalOut LED_RESET_PLACA(PA_5);
int cont_reset_placa = 0;

//================ Configura BNO055 =================
EstimadorAtitude estima_atitude;
Timer ciclo_BNO055;
Timer verifica_status;


//============= Configura Taxa de coleta =============
Timer t_dados; //Timer para taxa de aquisicao dos dados
Timer ciclo_programa; //Timer para verificar o tempo de ciclo do programa
int tx_aquisicao; //Quantos pontos serao plotados em um segundo
int t_amostragem; //valor do timer do Main



int main()
{

    pc.baud(115200); //Define a velocidade da porta USB


    t_dados.start();
    ciclo_programa.start();
    ciclo_BNO055.start();
    verifica_status.start();

    tx_aquisicao = 100;

    estima_atitude.config_imu();



    while(1) {

        estima_atitude.estima();
pc.printf("P= %f, Q= %f \r\n", estima_atitude.p, estima_atitude.q);
        wait_ms(20);
        

    }
}