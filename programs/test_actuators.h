#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

// Declara o mixer
Actuators act;



int main()
{
    
    pc.baud(115200); //Define a velocidade da porta USB
    
    act.actuate_servos(1.0, 1.0, 2.0);
    
    wait(5);

    act.actuate_servos(0.0, 0.0, 0.0);

    while(1) {

    }

}