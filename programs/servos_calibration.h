#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

// Imports
Actuators act;
Calibration calib;
InicializaPerifericos inicializa;


int main(){

    pc.baud(115200); //Define a velocidade da porta USB
    calib.config_calib_imu();
    inicializa.aviso_sonoro();
    wait(2);
    act.servo_test(100.0, 80.0, 2.0);
    wait(2);

    // while (calib.calibration != true) {
       while (true) {

        calib.phi_test_calib();

        // calib.calibra_servo_phi();
        // wait(1);
        // calib.calibra_servo_theta();
        // wait(1);

    }

    act.safe_state();

}