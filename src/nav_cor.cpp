#include "nav_alg.h"
#include "constants.h"
#include "nav_cor.h"

void Nav_correction::set_time_corr(uint32_t Time) 
{
  T = static_cast<float>(Time);
  w_s = (2 * M_PIf32) / T;
}

void Nav_correction::on_off_corr(bool Mode)
{ 

if (T < 0) {
    return;
}
  if (Mode) {
    k1 = 1.75f * w_s;
    k2 = ((2.15f * powf(w_s, 2)) / (G / R)) - 1;
    k3 = (powf(w_s, 3) / (G / R)) - 1.75f * w_s;
  } else {
    k1 = 0;
    k2 = 0;
    k3 = 0;
  }
}


const float& Nav_correction::pos_sns(size_t num) { return position_sns(num); }

const matrix::Vector2f& Nav_correction::pos_sns() { return position_sns; }

void Nav_correction::pos_sns(float vec_in[2]) {
  for (size_t i = 0; i < 2; i++) {
    vec_in[i] = position_sns(i);
  }
}

const float& Nav_correction::vel_sns(size_t num) { return velocity_sns(num); }

const matrix::Vector3f& Nav_correction::vel_sns() { return velocity_sns; }

void Nav_correction::vel_sns(float vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->velocity_sns(i);
  }
}

