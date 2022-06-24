#include "math.hpp"
#include "constants.h"
#include "nav_alg.h"
#include "nav_cor.h"

namespace NavA {
Nav::Nav(float p, float l, int f) {
  ns.position(0) = p;
  ns.position(1) = l;
  frequency = f;
  dt = 1 / float(frequency);
}

Nav::Nav(int f) {
  frequency = f;
  dt = 1 / float(frequency);
}

void Nav::set_pos(float p, float l) {
  ns.position(0) = p;
  ns.position(1) = l;
}

void Nav::puasson_equation(matrix::Vector3d &w_body) {
  dcm = dcm + (dcm * w_body.hat() - w_enu.hat() * dcm) * dt;
}

void Nav::euler_angles() {
  /*
          TODO: calculations of ns.r without ustd::sing copy constructor
  */
  double c0 = std::sqrt(std::pow(dcm(2, 0), 2.0) + std::pow(dcm(2, 2), 2.0));
  ns.rotation(0) = std::atan(dcm(2, 1) / c0);
  ns.rotation(1) = -std::atan(dcm(2, 0) / dcm(2, 2));
  ns.rotation(2) = std::atan2(dcm(0, 1), dcm(1, 1));
}

void Nav::get_prh(float prh[3]) {
  double c0 = std::sqrt(std::pow(dcm(2, 0), 2.0) + std::pow(dcm(2, 2), 2.0));
  float teta =   static_cast<float>(std::atan(dcm(2, 1) / c0));
  float gamma =  static_cast<float>(-std::atan(dcm(2, 0) / dcm(2, 2)));
  float psi =    static_cast<float>(std::atan2(dcm(0, 1), dcm(1, 1)));

  prh[0] = teta;
  prh[1] = gamma;
  prh[2] = psi;
}

void Nav::acc_body_enu(matrix::Vector3d &a_body) {
  a_enu = dcm * a_body;
}

void Nav::speed() {
  ns.velocity(0) =
      ns.velocity(0) + (a_enu(0)
      + (U * std::sin(ns.position(0)) + w_enu(2)) * ns.velocity(1)
      - co.gnss_coeff[1] * G * co.dpos(1) * std::cos(ns.position(0)) ) * dt;

  ns.velocity(1) =
      ns.velocity(1) + (a_enu(1)
      - (U * std::sin(ns.position(0)) + w_enu(2)) * ns.velocity(0)
      - co.gnss_coeff[1] * G * co.dpos(0)) * dt;
}

void Nav::coordinates() {
  // Latitude
  ns.position(0) = ns.position(0) + (ns.velocity(1) / (R + H) - co.gnss_coeff[0] * co.dpos(0)) * dt;
  // Longitude
  ns.position(1) = ns.position(1) + (ns.velocity(0) / ((R + H) * std::cos(ns.position(0))) - co.gnss_coeff[0] * co.dpos(1)) * dt;
}

void Nav::ang_velocity_body_enu() {
  w_enu(0) = -ns.velocity(1) / (R + H)
  - (co.gnss_coeff[2] * co.dpos(0))
  - co.integ_rad_c*a_enu(1);

  w_enu(1) = ns.velocity(0) / (R + H)
  + U * std::cos(ns.position(0))
  + co.gnss_coeff[2] * co.dpos(1) * std::cos(ns.position(0))
  + co.integ_rad_c*a_enu(0);

  w_enu(2) = (ns.velocity(0) / (R + H)) * std::tan(ns.position(0))
  + U * std::sin(ns.position(0));
}

void Nav::alignment(float roll, float pitch, float yaw) {
  float psi = yaw;
  float teta = pitch;
  float gamma = roll;

  float sp = std::sin(psi);
  float st = std::sin(teta);
  float sg = std::sin(gamma);

  float cp = std::cos(psi);
  float ct = std::cos(teta);
  float cg = std::cos(gamma);

  alignment(st, ct, sg, cg, sp, cp);
}

void Nav::alignment(float st, float ct, float sg, float cg, float sp,
                    float cp) {
  dcm(0, 0) = cp * cg + sp * st * sg;
  dcm(0, 1) = sp * ct;
  dcm(0, 2) = cp * sg - sp * st * cg;
  dcm(1, 0) = -sp * cg + cp * st * sg;
  dcm(1, 1) = cp * ct;
  dcm(1, 2) = -sp * sg - cp * st * cg;
  dcm(2, 0) = -ct * sg;
  dcm(2, 1) = st;
  dcm(2, 2) = ct * cg;
}

void Nav::alignment(float ax, float ay, float az, float yaw) {
  float psi = yaw;

  float A = std::sqrt(std::pow(ax, 2.0f) + std::pow(az, 2.0f));

  float st = ay / static_cast<float>(G);
  float sg = -1 * ax / A;

  float ct = A / static_cast<float>(G);
  float cg = az / A;

  float sp = std::sin(psi);
  float cp = std::cos(psi);

  alignment(st, ct, sg, cg, sp, cp);
}

void Nav::iter(D_IN const &in) {
  if (!in.imu.acc.has_value() || !in.imu.gyr.has_value()) {
    return;
  }
  
  if (in.gnss.pos.has_value()) {
    co.dpos = ns.position - in.gnss.pos.value();
  } else {
    co.on_off_gnss_corr(false);
  }

  auto a = in.imu.acc.value();
  auto g = in.imu.gyr.value();

  acc_body_enu(a);
  speed();
  if (!co.rad_c)
    ang_velocity_body_enu();
  else {
    w_enu(0) = a_enu(1) * -co.rad_corr_c;
    w_enu(1) = a_enu(0) * co.rad_corr_c;
    w_enu(2) = (ns.velocity(0) / (R + H)) * std::tan(ns.position(0)) + U * std::sin(ns.position(0));
  }
  puasson_equation(g);
  if (i % 5 == 0)
  {
    dcm.renormalize();
    i = 0;
  } else {
    i++;
  }
  coordinates();
  euler_angles();
}
}

/* TODO:
        add a function to convert the magnetometer
        yaw from body to enu
vec_enu Nav::mag_to_enu()
*/
