#include "nav_alg.h"
#include "constants.h"
#include "nav_cor.h"

void Nav_correction::set_time_corr(uint32_t Time) 
{
  T = Time;
}

void Nav_correction::on_off_corr(bool Mode)
{ 

if (T == 0) {
    return;
}
  if (Mode) {
    w_s = (2 * Pi) / T;
    k1 = 1.75 * w_s;
    k2 = ((2.15 * pow(w_s,2)) / (G / R)) - 1;
    k3 = (pow(w_s, 3) / (G / R)) - 1.75 * w_s;
  } else {
    k1 = 0;
    k2 = 0;
    k3 = 0;
  }
}
