#include "mbed.h"
#include "imports.h"
#include <cstdio>

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

// Imports
Actuators act;
Calibration calib;
Initialization init;


int main(){

    pc.baud(115200); //Define a velocidade da porta USB
    calib.config_calib_imu();
    init.arm();
    wait(2);
    act.servo_test(100.0, 70.0, 1.0);
    wait(2);

    // while (calib.calibration != true) {
       while (true) {
        // calib.estimate();
        // pc.printf("PHI = %f, THETA = %F\r\n", calib.Phi*180/pi, calib.Theta*180/pi);

        calib.phi_test_calib();

        //  calib.calibra_servo_phi();
        //  wait(1);
        //  calib.calibra_servo_theta();
        //  wait(1);

    }

    act.safe_state();

}