#ifndef analysis_api_h__
#define analysis_api_h__

#include <memory>

#include "constants.h"
#include "nav_alg.h"

/*
  3 component vector of float numbers
*/
typedef struct {
  float data[3];
} Vector3f;

/*
  2 component vector of float numbers
*/
typedef struct {
  float data[2];
} Vector2f;

typedef struct {
  const size_t size;

  /*
    array of accelerometer data.
    0 elem - x component
    1 elem - y component
    2 elem - z component
  */
  const Vector3f *const a;
  /*
    array of gyroscope data.
    0 elem - x component
    1 elem - y component
    2 elem - z component
  */
  const Vector3f *const g;
} SENS_IN;

typedef struct {
  const size_t size;
  /*
    array of rotation angles
    0 elem - pitch
    1 elem - roll
    2 elem - yaw
  */
  const Vector3f *const pry;
  /*
    array of coordinates
    0 elem - lat
    1 elem - lon
  */
  const Vector2f *const coord;
  /*
    array of velocities in ENU frame
    0 elem - east component
    1 elem - north component
  */
  const Vector2f *const vel;
} NAV_OUT;

class NavIface {
 private:
  Nav nav_alg;

 public:
  NavIface(float lat, float lon, int frequency);
  ~NavIface();

  NAV_OUT solution(SENS_IN sensors_data);

  Nav* nav() { return &this->nav_alg; };

};

extern "C" {

NavIface *NavIface_new(float lat, float lon, int frequency) {
  return new NavIface(lat, lon, frequency);
}

//void i_get_prh(NavIface *i, vec_body *v) { i->nav_get_prh(v); }

float i_get_u(void) { return U; }

float i_get_g(void) { return G; }

NAV_OUT i_solution(NavIface* const i, SENS_IN sensors_data) {
  return i->solution(sensors_data);
}

Nav* i_nav(NavIface* const i) {
  return i->nav();
}

void n_alignment_rph(Nav *n, float roll, float pitch, float yaw) {
  n->alignment(roll, pitch, yaw);
}

void n_alignment_acc(Nav *n, float ax_mean, float ay_mean, float az_mean,
                     float yaw) {
  n->alignment(ax_mean, ay_mean, az_mean, yaw);
}

void n_alignment_cos(Nav* n, float st, float ct, float sg, float cg,
                     float sp, float cp) {
  n->alignment(st, ct, sg, cg, sp, cp);
}

void n_iter(Nav *n, const float acc[3], const float gyr[3]) {
  n->iter(acc, gyr);
}

void n_pry(Nav *n, float rot[3]) {
  n->sol().rot(rot);
}

void n_vel(Nav *n, float vel[3]) {
  n->sol().vel(vel);
}

void n_pos(Nav *n, float coord[2]) {
  n->sol().pos(coord);
}

void n_align_prh(Nav *n, float prh[3]) {
  n->get_prh(prh);
}

}

#endif
