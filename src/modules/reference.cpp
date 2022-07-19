#include "reference.h"
#include "mbed.h"


// Classe do construtor
Reference::Reference() {
}

void Reference::ref_generator(double input_signal1, double input_signal2) {

  x_ref_theta[0] = input_signal1;
  x_ref_theta[1] = 0.0;             // Very small number in matlab
  x_ref_theta[2] = 0.0;             // Very small number in matlab
  u_ref_theta[0] = (input_signal1 * 1068) / 1339;

  x_ref_phi[0] = input_signal2;
  x_ref_phi[1] = 0.0;             // Very small number in matlab
  x_ref_phi[2] = 0.0;             // Very small number in matlab
  u_ref_phi[0] = (input_signal2 * 1068) / 1339;   // Will change

}
