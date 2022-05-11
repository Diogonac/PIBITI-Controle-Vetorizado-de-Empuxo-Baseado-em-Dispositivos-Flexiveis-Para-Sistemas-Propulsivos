#include "mbed.h"
#include "imports.h"


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1, 115200); // Comunicação com USB TX, RX

// Controller ans Estimator objects
Actuators act;
Initialization init;
EstimadorAtitude att_est;
ControladorAtitude att_cont;
VerticalEstimator ver_est;
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
    if(input_wave.read() <= 5){
    z_r = Z1;
    }
    if(input_wave.read() > 5){
        z_r = Z2; 
    }
    if(input_wave.read() >= 10){
        input_wave.reset();
  }
}

// Main program
int main() {

  // Set references
  float phi_r = 0.0;
  float theta_r = 0.0;
  float P_r = 0.0;
  float Q_r = 0.0;

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

  while (abs(att_est.Phi) <= pi / 4.0 && abs(att_est.Theta) <= pi / 4.0 && abs(att_est.P) <= 4.0 * pi && abs(att_est.Q) <= 4.0 * pi && abs(att_est.R) <= 4.0 * pi && init.valve_flag == 1) {
    if (flag) {
      flag = false;
      att_est.estimate();
    //   ver_est.predict(ver_cont.f_z);
        ver_est.predict(0.0);

      if (flag_range) {
        flag_range = false;
        ver_est.correct(att_est.Phi, att_est.Theta);
        //ver_est.correct(0.0, 0.0);

      }
      input_ref(0.3, 0.3);
      ver_cont.control(z_r, ver_est.z, ver_est.w);
      att_cont.control(phi_r, theta_r, att_est.Phi, att_est.Theta, att_est.P, att_est.Q, P_r, Q_r);

      act.actuate_servos(att_cont.f_x, att_cont.f_y, ver_cont.f_z);
      act.actuate_valve(att_cont.f_x, att_cont.f_y, ver_cont.f_z);
    
      pc.printf("%f %f %f %f %f %f %f\n", ver_cont.f_z, att_cont.f_y, att_cont.f_x, att_est.Phi, att_est.Theta, ver_est.z, z_r);

    }
  }
}
