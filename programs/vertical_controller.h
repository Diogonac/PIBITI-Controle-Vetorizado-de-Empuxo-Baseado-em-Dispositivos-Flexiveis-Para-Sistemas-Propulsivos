#include "mbed.h"
#include "imports.h"


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1, 115200); // Comunicação com USB TX, RX

// Controller ans Estimator objects
Actuators act;
Initialization init;
VerticalEstimator ver_est;
EstimadorAtitude att_est;
VerticalController ver_cont;

// Time objects
Timer input_wave;
// Ticker objects
Ticker tic, tic_range;

// Interrupt flag and counter variables
bool flag, flag_range;

double z_r;

// Callback functions
void callback() { flag = true; }
void callback_range() { flag_range = true; }

void input_ref(double Z1, double Z2){
    if(input_wave.read() <= 2.5){
    z_r = Z1;
    }
    if(input_wave.read() > 2.5){
        z_r = Z2; 
    }
    if(input_wave.read() >= 5){
        input_wave.reset();
  }
}

// Main program
int main() {

  // Set references
  float phi_r = 0.0;
  float theta_r = 0.0;
  float psi_r = 0.0;

  // Initialize estimators objects
  ver_est.init(); 
  att_est.init();
  act.config_dac();

  // Initialize interrupts
  tic.attach(&callback, dt);
  tic_range.attach(&callback_range, dt_range);
  

  // Arm the sistem to initialization 
  init.arm();
  input_wave.start();

  //while (abs(att_est.Phi) <= pi / 4.0 && abs(att_est.Theta) <= pi / 4.0 && abs(att_est.P) <= 4.0 * pi && abs(att_est.Q) <= 4.0 * pi && abs(att_est.R) <= 4.0 * pi) {
    while (init.valve_flag == 1){
    if (flag) {
      flag = false;
      att_est.estimate();
      ver_est.predict(ver_cont.f_z);
      if (flag_range) {
        flag_range = false;
        // ver_est.correct(att_est.Phi, att_est.Theta);
        ver_est.correct(0.0, 0.0);

      }
      input_ref(0.4, 0.2);
      ver_cont.control(z_r, ver_est.z, ver_est.w);
    //   ver_cont.feed_foward(ver_cont.f_z, ver_est.w, f_max, K1, K2, K3, f_stiction, f_coulomb, z_r, ver_est.z);
        ver_cont.feed_foward(ver_cont.f_z, ver_est.w, 0, 0, 0, 0, 0, f_coulomb, z_r, ver_est.z);

      //att_cont.control(phi_r, theta_r, psi_r, att_est.phi, att_est.theta, att_est.psi, att_est.p, att_est.q, att_est.r);
      act.actuate_valve(0.0, 0.0, ver_cont.uout);
    //   act.actuate_valve(0.0, 0.0, m*g);
    //   pc.printf("F_stiction = %f\r\n", f_stiction);
       pc.printf("%f %f\n", ver_est.z, z_r);
    //   pc.printf("%f %f %f %f %f %f\n", ver_cont.uff, ver_cont.f_z, ver_cont.uout, ver_est.w, z_r, ver_est.z);


    }
  }
}