#include "direct_one.h"
#include "mbed.h"


// Classe do construtor
DirectOne::DirectOne(double b0, double b1, double b2, double b3, double a0, double a1, double a2, double a3){

  b[0] = b0;
  b[1] = b1;
  b[2] = b2;
  b[3] = b3;

  a[0] = a0;
  a[1] = a1;
  a[2] = a2;
  a[3] = a3;
};

inline void DirectOne::shift(double new_in, double new_out) {

  in_m[3] = in_m[2];
  in_m[2] = in_m[1];
  in_m[1] = new_in;

  out_m[3] = out_m[2];
  out_m[2] = out_m[1];
  out_m[1] = new_out;
};

double DirectOne::update(double input) {

  double inter_b = b[0] * input + b[1] * in_m[1] + b[2] * in_m[2] + b[3] * in_m[3];
  double out = inter_b - a[1] * out_m[1] - a[2] * out_m[2] - a[3] * out_m[3];

  shift(input, out);

  return out;
};
