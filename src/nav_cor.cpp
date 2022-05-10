#include "nav_alg.h"
#include "constants.h"
#include "nav_cor.h"


void Nav_correction::set_time_gnss_corr(float Time) 
{
  T = Time;
  w_s = (2 * M_PIf32) / T;
}

void Nav_correction::on_off_gnss_corr(bool Mode)
{ 

if (T < 0) {
    return;
}
  if (Mode) {
    gnss_coeff[0] = 1.75f * w_s;
    gnss_coeff[1] = ((2.15f * powf32(w_s, 2)) / (G / R)) - 1;
    gnss_coeff[2] = (powf32(w_s, 3) / (G / R)) - 1.75f * w_s;
  } else {
    gnss_coeff[0] = 0;
    gnss_coeff[1] = 0;
    gnss_coeff[2] = 0;
  }
}
