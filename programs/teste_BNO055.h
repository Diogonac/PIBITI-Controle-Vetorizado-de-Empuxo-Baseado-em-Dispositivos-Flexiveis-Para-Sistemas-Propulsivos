#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1, 115200); //Comunicação com USB TX, RX

// Imports
AttitudeObserver obs;
Initialization init;

// Ticker para taxa de amostragem
Ticker amostragem;

// Flag de segurança
bool flag;

void callback(void);

int main()
{

    pc.baud(115200); //Define a velocidade da porta USB
    
    obs.initIMU();

    //Definição da taxa de amostragem
    amostragem.attach(&callback, dt);

    // Arm the sistem to initialization
    init.arm();
    
    while(1){
            if(flag) {
                
                flag = false;

                obs.readIMU();

                pc.printf("Phi= %f, Theta= %f, Psi= %f ", (180.0*obs.Phi/pi), (180.0*obs.Theta/pi), (180.0*obs.Psi/pi));
                pc.printf("GX= %f, GY= %f, GZ= %f \r\n", obs.P, obs.Q, obs.R);

            }

    }


    }



void callback()
{
    flag = true;

}

    