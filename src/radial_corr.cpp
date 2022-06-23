#include "nav_cor.h"

namespace NavA {
void Nav_correction::on_off_radial_corr(bool Mode) {
  if (Mode) {
      rad_corr_c = r_c;
  } else {
      rad_corr_c = 0;
  }
  rad_c = Mode;
}

void Nav_correction::set_k_radial_corr(float k) {
    r_c = k;
}
}