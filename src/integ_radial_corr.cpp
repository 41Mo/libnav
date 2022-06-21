#include "nav_cor.h"

namespace NavA {
void Nav_correction::on_off_int_rad_corr(bool Mode) {
  if (Mode) {
      integ_rad_c = ir_c;
  } else {
      integ_rad_c = 0;
  }
}

void Nav_correction::set_k_integ_rad_corr(float k) {
    ir_c = k;
}
}