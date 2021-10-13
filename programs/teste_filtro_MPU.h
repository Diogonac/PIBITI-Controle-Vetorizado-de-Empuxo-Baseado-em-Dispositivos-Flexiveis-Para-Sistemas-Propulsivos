#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

// Imports
Mixer mixer;
ControladorAtitude cont_atitude;
EstimadorAtitude estima_atitude;
InicializaPerifericos inicializa;

// Ticker para taxa de amostragem
Ticker amostragem;

Timer tempo;

// Flag de segurança
bool flag;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r;


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
    mixer.config_servos();
    estima_atitude.config_imu();
    mixer.config_MPU();
    
    inicializa.verifica(mixer.verifica_servos, estima_atitude.verifica_imu, mixer.verifica_MPU);
    

    //Definição da taxa de amostragem
    amostragem.attach(&callback, dt);

    //mixer.calibra_servo_MPU();

    while(inicializa.sistema_verificado) {

            if(flag) {
                
                flag = false;

                mixer.estima_MPU();


            }
            
            tempo.reset();

        }
}

void callback()
{
    flag = true;

}