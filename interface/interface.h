#ifndef analysis_api_h__
#define analysis_api_h__

#include <memory>

#include "constants.h"
#include "nav_alg.h"
class NavIface {
 private:
  NavA::Nav nav_alg;

 public:
  NavIface(float lat, float lon, int frequency);
  ~NavIface();

  NavA::Nav* nav() { return &this->nav_alg; };
};

extern "C" {

NavIface *NavIface_new(float lat, float lon, int frequency) {
  return new NavIface(lat, lon, frequency);
}

//void i_get_prh(NavIface *i, vec_body *v) { i->nav_get_prh(v); }

float i_get_u(void) { return static_cast<float>(NavA::U); }

float i_get_g(void) { return static_cast<float>(NavA::G); }

NavA::Nav* i_nav(NavIface* const i) {
  return i->nav();
}

void n_alignment_rph(NavA::Nav *n, float roll, float pitch, float yaw) {
  n->alignment(roll, pitch, yaw);
}

void n_alignment_acc(NavA::Nav *n, float ax_mean, float ay_mean, float az_mean,
                     float yaw) {
  n->alignment(ax_mean, ay_mean, az_mean, yaw);
}

void n_alignment_cos(NavA::Nav* n, float st, float ct, float sg, float cg,
                     float sp, float cp) {
  n->alignment(st, ct, sg, cg, sp, cp);
}

void n_iter(NavA::Nav *n, const double acc[3], const double gyr[3]) {
  using namespace NavA;
  D_IN d {
    D_IMU{
      matrix::Vector3d(acc),matrix::Vector3d(gyr)
    },
    D_GNSS{}
  };

  n->iter(d);
}

void n_iter_gnss(NavA::Nav *n, const double acc[3], const double gyr[3], double gnss_pos[2]) {
  using namespace NavA;
  D_IN d {
    D_IMU {matrix::Vector3d(acc),matrix::Vector3d(gyr)},
    D_GNSS{matrix::Vector2d(gnss_pos)}
  };
  n->iter(d);
}

void n_time_corr(NavA::Nav *n, float time) {
  n->cor().set_time_gnss_corr(time);
}

void n_mode_corr(NavA::Nav *n, bool mode) {
  n->cor().on_off_gnss_corr(mode);
}

void n_set_pos(NavA::Nav *n, float lat, float lon) {
  n->set_pos(lat, lon);
}

void n_pry(NavA::Nav *n, double rot[3]) {
  n->sol().rot(rot);
}

void n_vel(NavA::Nav *n, double vel[3]) {
  n->sol().vel(vel);
}

void n_pos(NavA::Nav *n, double coord[2]) {
  n->sol().pos(coord);
}

void n_align_prh(NavA::Nav *n, float prh[3]) {
  n->get_prh(prh);
}

float n_corr_k(NavA::Nav *n, int num) {
  return n->cor().k(num);
}

void toggle_rad_c(NavA::Nav *n, bool Mode) {
  n->cor().on_off_radial_corr(Mode);
}

void rad_set_k(NavA::Nav *n, float k) {
  n->cor().set_k_radial_corr(k);
}

void toggle_integ_rad_c(NavA::Nav *n, bool Mode) {
  n->cor().on_off_int_rad_corr(Mode);
}

void integ_rad_set_k(NavA::Nav *n, float k) {
  n->cor().set_k_integ_rad_corr(k);
}
}
#endif
