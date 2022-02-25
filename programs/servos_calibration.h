#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

// Imports
Actuators act;
Calibration calib;

int main(){

    pc.baud(115200); //Define a velocidade da porta USB

    // while (calib.calibration != true) {
       while (true) {

        calib.calibra_servo_phi();
        wait(1);
        calib.calibra_servo_theta();
        wait(1);

    }

    act.safe_state();

}