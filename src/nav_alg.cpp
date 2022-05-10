#include "nav_alg.h"
#include "constants.h"
#include "nav_cor.h"

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

void Nav::puasson_equation(matrix::Vector3f &w_body) {
  dcm = dcm + (dcm * w_body.hat() - w_enu.hat() * dcm) * dt;
}

void Nav::euler_angles() {
  /*
          TODO: calculations of ns.r without usinf32g copy constructor
  */
  float c0 = sqrtf32(powf32(dcm(2, 0), 2) + powf32(dcm(2, 2), 2));
  ns.rotation(0) = atanf32(dcm(2, 1) / c0);
  ns.rotation(1) = -atanf32(dcm(2, 0) / dcm(2, 2));
  ns.rotation(2) = atan2f(dcm(0, 1), dcm(1, 1));
}

void Nav::get_prh(float prh[3]) {
  float c0 = sqrtf32(powf32(dcm(2, 0), 2) + powf32(dcm(2, 2), 2));
  float teta = atanf32(dcm(2, 1) / c0);
  float gamma = -atanf32(dcm(2, 0) / dcm(2, 2));
  float psi = atan2f(dcm(0, 1), dcm(1, 1));

  prh[0] = teta;
  prh[1] = gamma;
  prh[2] = psi;
}

void Nav::acc_body_enu(matrix::Vector3f &a_body) { a_enu = dcm * a_body; }

void Nav::speed() {
  ns.velocity(0) =
      ns.velocity(0) + (a_enu(0) + (U * sinf32(ns.position(0)) + w_enu(2)) * ns.velocity(1) - co.gnss_coeff[0] * (ns.velocity(0) - co.velocity_sns(0))) * dt;
  ns.velocity(1) =
      ns.velocity(1) + (a_enu(1) - (U * sinf32(ns.position(0)) + w_enu(2)) * ns.velocity(0) - co.gnss_coeff[0] * (ns.velocity(1) - co.velocity_sns(1))) * dt;
}

void Nav::coordinates() {
  // Latitude
  ns.position(0) = ns.position(0) + (ns.velocity(1) / (R + H) - co.gnss_coeff[2] * (ns.position(0) - co.position_sns(0))) * dt;
  // Longitude
  ns.position(1) = ns.position(1) + (ns.velocity(0) / ((R + H) * cosf32(ns.position(0))) - co.gnss_coeff[2] * (ns.position(1) - co.position_sns(1))) * dt;
}

void Nav::ang_velocity_body_enu() {
  w_enu(0) = -ns.velocity(1) / (R + H) - (co.gnss_coeff[1] / (R + H)) * (ns.velocity(1) - co.velocity_sns(1));
  w_enu(1) = ns.velocity(0) / (R + H) + U * cosf32(ns.position(0)) + (co.gnss_coeff[1] / (R + H)) * (ns.velocity(0) - co.velocity_sns(0));
  w_enu(2) = (ns.velocity(0) / (R + H)) * tanf32(ns.position(0)) + U * sinf32(ns.position(0));
}

void Nav::alignment(float roll, float pitch, float yaw) {
  float psi = yaw;
  float teta = pitch;
  float gamma = roll;

  float sp = sinf32(psi);
  float st = sinf32(teta);
  float sg = sinf32(gamma);

  float cp = cosf32(psi);
  float ct = cosf32(teta);
  float cg = cosf32(gamma);

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

  float A = sqrtf32(powf32(ax, 2) + powf32(az, 2));

  float st = ay / G;
  float sg = -1 * ax / A;

  float ct = A / G;
  float cg = az / A;

  float sp = sinf32(psi);
  float cp = cosf32(psi);

  alignment(st, ct, sg, cg, sp, cp);
}

void Nav::iter(matrix::Vector3f &acc, matrix::Vector3f &gyr) {
  acc_body_enu(acc);
  ang_velocity_body_enu();
  dcm.renormalize();
  puasson_equation(gyr);
  speed();
  euler_angles();
  coordinates();
}

void Nav::iter(const vec_body &acc, const vec_body &gyr) {
  auto a = matrix::Vector3f(acc.X, acc.Y, acc.Z);
  auto g = matrix::Vector3f(gyr.X, gyr.Y, gyr.Z);
  iter(a, g);
}

void Nav::iter(const float acc[3], const float gyr[3]) {
  auto a = matrix::Vector3f(acc);
  auto g = matrix::Vector3f(gyr);
  iter(a, g);
}

void Nav::iter(const float acc[3], const float gyr[3], const float gnss_vel[3], const float gnss_pos[2]) {
  auto a = matrix::Vector3f(acc);
  auto g = matrix::Vector3f(gyr);
  co.velocity_sns = matrix::Vector3f(gnss_vel);
  co.position_sns = matrix::Vector2f(gnss_pos);
  iter(a, g);
}

/* TODO:
        add a function to convert the magnetometer
        yaw from body to enu
vec_enu Nav::mag_to_enu()
*/
