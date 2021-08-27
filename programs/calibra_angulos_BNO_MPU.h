#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

// Imports
ControladorAtitude cont_atitude;
InicializaPerifericos inicializa;
EstimadorAtitudeMPU estima_atitudeMPU;

// Ticker para taxa de amostragem
Ticker amostragem;

Timer tempo;

// Flag de segurança
bool flag;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r;

double pos = 0.0;


// Tempo de amostragem
const double dt = 0.002; // 500 Hz

void callback(void);

int main()
{
    tempo.start();

    pc.baud(115200); //Define a velocidade da porta USB

    // Defino o valor das veriáveis de referência
    phi_r = 0.0;
    theta_r = 0.0;
    p_r = 0.0;
    q_r = 0.0;

    // Configuração dos periféricos
    estima_atitudeMPU.config_MPU();

    //Definição da taxa de amostragem
    amostragem.attach(&callback, dt);

    //Calibração inicial dos servos
    estima_atitudeMPU.calibra_angulo(1.0, 0.15);

    while(1) {


        //estima_atitudeMPU.estima_MPU();

        // estima_atitudeMPU.movimenta_servo(90.0, 5.0);
        // estima_atitudeMPU.movimenta_servo(100.0, 5.0);

        estima_atitudeMPU.calibra_angulo(1.5, 0.15);

        
    
    //pc.printf("PITCH= %f, ROLL= %f, P= %f, q= %f, r= %f\r\n", estima_atitude.Theta, estima_atitude.Phi, estima_atitude.P, estima_atitude.Q, estima_atitude.R);
}

}


void callback()
{
    flag = true;
}
