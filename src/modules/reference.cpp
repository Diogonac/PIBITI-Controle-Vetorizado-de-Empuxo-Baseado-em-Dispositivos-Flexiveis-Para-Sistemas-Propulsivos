#include "reference.h"
#include "mbed.h"


// Classe do construtor
Reference::Reference() {
}

void Reference::ref_angle(double input_signal1, double input_signal2) {

  ref_theta[0] =  0.89324503095650709116881671434385*input_signal1;
  ref_theta[1] =  0.21552619177621634705133146781009*input_signal1;             // Very small number in matlab
  u_ref_theta = 0.87843961009708904086323855153751*input_signal1;

  ref_phi[0] = 0.0014222727919214905418859337220879*input_signal2;
  ref_phi[1] = 0.21552619177621634705133146781009*input_signal2;             // Very small number in matlab
  u_ref_phi = 1.6171241644147347461243066420139*input_signal2;   // Will change
}

void Reference::ref_vertical(double input_signal1){

  ref_z[0] = input_signal1;
  ref_z[1] = 0.0;
  u_ref_z = 0.0;

}
