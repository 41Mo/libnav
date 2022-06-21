#pragma once
#include "math.hpp"

namespace NavA {
class Nav_correction {
 public:
  void on_off_gnss_corr(bool Mode);
  void set_time_gnss_corr(float Time);
  void corr_coef(float dphi, float dlam);
  float k(int num) { return g_c[num]; };
  void on_off_radial_corr(bool Mode);
  void set_k_radial_corr(float k);
  void on_off_int_rad_corr(bool Mode);
  void set_k_integ_rad_corr(float k);

 private:
  matrix::Vector2f dpos;
  float w_s{0.0f};
  float T;
  float gnss_coeff[3]{0, 0, 0};
  float rad_corr_c{0};
  float integ_rad_c{0};
  float rc_on{-1};
  bool rad_c{false};

  float g_c[3]{0, 0, 0};
  float r_c{1};
  float ir_c{0};
 protected:
  friend class Nav;
  Nav_correction() {}
};
}