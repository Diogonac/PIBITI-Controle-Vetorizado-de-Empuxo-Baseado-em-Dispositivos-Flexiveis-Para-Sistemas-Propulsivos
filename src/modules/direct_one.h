#ifndef direct_one_h
#define direct_one_h

// Class observer
class DirectOne

{
public:
  // Classe do construtor
  DirectOne(double b0, double b1, double b2, double b3, double a0, double a1, double a2, double a3);

  // Main functions
  double update(double input);
  
  // Auxiliar variables
  double b[4];
  double a[4];
  double in_m[4];
  double out_m[4];

private:
  inline void shift(double new_in, double new_out);

};

#endif
