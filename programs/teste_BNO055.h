#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1, 115200); //Comunicação com USB TX, RX

// Imports
AttitudeEstimator att_est;

// Ticker para taxa de amostragem
Ticker amostragem;

// Flag de segurança
bool flag;

void callback(void);

int main()
{

    pc.baud(115200); //Define a velocidade da porta USB
    
    att_est.init();

    //Definição da taxa de amostragem
    amostragem.attach(&callback, dt);

    while(1){
            if(flag) {
                
                flag = false;

                att_est.estimate();

                    printf("Phi= %f, Theta= %f, Psi= %f ", (180.0*att_est.Phi/pi), (180.0*att_est.Theta/pi), (180.0*att_est.Psi/pi));
                    printf("GX= %f, GY= %f, GZ= %f \r\n", att_est.P, att_est.Q, att_est.R);

            }

    }


    }



void callback()
{
    flag = true;

}

    