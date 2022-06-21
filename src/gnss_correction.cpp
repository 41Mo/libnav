#include "nav_alg.h"
#include "constants.h"
#include "nav_cor.h"

namespace NavA {

void Nav_correction::set_time_gnss_corr(float Time) 
{
  if (Time < 1e-8f ) {
    printf("Correction: Worng Time:%f", T);
    exit(-2);
  }
  T = Time;
  w_s = (2.0f * static_cast<float>(M_PI)) / T;
  g_c[0] = 1.75f * w_s;
  g_c[1] = ((2.15f * std::pow(w_s, 2.0f)) / (G / R)) - 1;
  g_c[2] = (std::pow(w_s, 3.0f) / (G / R)) - 1.75f * w_s;
}

void Nav_correction::on_off_gnss_corr(bool Mode)
{ 
  if (Mode) {
    gnss_coeff[0] = g_c[0];
    gnss_coeff[1] = g_c[1];
    gnss_coeff[2] = g_c[2];
  } else {
    gnss_coeff[0] = 0;
    gnss_coeff[1] = 0;
    gnss_coeff[2] = 0;
  }
}
}
