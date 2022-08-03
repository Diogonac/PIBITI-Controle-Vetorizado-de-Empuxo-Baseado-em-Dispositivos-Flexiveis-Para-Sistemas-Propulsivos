#include "horizontal_estimator.h"
#include "mbed.h"

// Classe do construtor
HorizontalEstimator::HorizontalEstimator()
    : flow(E_MOSI, E_MISO, E_SCK, E_CS1) {
  // Observer variables init
  x_hat = 0.0;
  y_hat = 0.0;

  vel_x = 0.0;
  vel_x_hat = 0.0;
  vel_y = 0.0;
  vel_y_hat = 0.0;
}

void HorizontalEstimator::estimate(double phi, double theta, double f_x, double f_y) {

//   vel_x_hat = vel_x_hat + (g * theta + f_x / m) * dt;
//   vel_y_hat = vel_y_hat + (-g * phi + f_y / m) * dt;

//   if (abs(vel_x_hat) > 0.02) {

//     vel_x_hat = vel_x_hat + (g * theta + f_x / m) * dt;

//   } else {

//     vel_x_hat = 0.0;
//   }

//     if (abs(vel_y_hat) > 0.02) {

//     vel_y_hat = vel_y_hat + (-g * phi + f_y / m) * dt;

//   } else {

//     vel_y_hat = 0.0;
//   }

  x_hat = x_hat + vel_x_hat * dt;
  y_hat = y_hat + vel_y_hat * dt;

  estimated_x[0] = x_hat;

  estimated_y[0] = y_hat;
}

void HorizontalEstimator::init() { flow.init(); }

void HorizontalEstimator::read(double phi, double theta, double p, double q, double z) {

  flow.read();
  float den = cos(phi) * cos(theta);

  if (den > 0.5) {

    float d = z / den;
    vel_x = (sigma * flow.px + q) * d;
    vel_y = (sigma * flow.py - p) * d;

    vel_x_hat = vel_x_hat + l_hor * dt * (vel_x - vel_x_hat);
    vel_y_hat = vel_y_hat + l_hor * dt * (vel_y - vel_y_hat);

    estimated_x[1] = vel_x_hat;
    estimated_y[1] = vel_y_hat;


  }
}
