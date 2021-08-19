#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

// Declara o mixer
Mixer mixer;



int main()
{
    
    pc.baud(115200); //Define a velocidade da porta USB
    
    mixer.actuate(1.0, 1.0, 2.0);
    
    wait(5);

    mixer.actuate(0.0, 0.0, 0.0);

    while(1) {

    }

}