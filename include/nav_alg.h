#pragma once
#include "math.hpp"
#include "nav_solution.h"
#include "nav_cor.h"
#include "input_structure.h"
#include "calib.h"

namespace NavA {
class Nav {
 public:
  Nav() {};
  /*
          Constructor based on current coordinates and frequency.
  */
  Nav(float phi, float lambda, int frequency);

  /*
          Constructor based on frequency.
  */
  Nav(int frequency);

  /*
          Set current position in ENU frame.
  */
  void set_pos(float phi, float lambda);

  /*
          Alignment based on 3 angles of rotation
  */
  void alignment(float roll, float pitch, float yaw);

  /*
          Alignment based on mean values of acceleration along 3 axis in body
     frame and yaw.
  */
  void alignment(float ax_mean, float ay_mean, float az_mean, float yaw);

  /*
          Alignment based on sin and cos of 3 angles of rotation.
  */
  void alignment(float st, float ct, float sg, float cg, float sp, float cp);

  /*
          Do 1 iteration over acc and gyr data.
  */
  void iter(D_IN &input);

  /*
  */
  inline void set_calib_file(const char* file, Calib::calib_type t) {
    cal = Calib(file, t);
  };

  /*
          Get solution for current iteration.
  */
  Nav_solution &sol() { return ns; }
  Nav_correction &cor() { return co; }

  /*
  Special function just for analysis purposes.
  Primarly used to check if alignment was successfull.
  */
  void get_prh(float prh[3]);

 private:
  void puasson_equation(matrix::Vector3d &w_body);
  void euler_angles();
  void acc_body_enu(matrix::Vector3d &a_body);
  void speed();
  void coordinates();
  void ang_velocity_body_enu();
  void norm_row();
  void norm_column();
  void normalize();
  void set_corr_mode(int Time, bool Mode);

  /*
          All variables are represented in SI.
  */

  /*
          Direction cosine matrix
  */
  matrix::Dcmd dcm;

  float H{0};        // height above ground
  int frequency{0};  // operation frequency
  float dt{0};
  int i{0};

  matrix::Vector3d w_enu{0, 0, 0};
  matrix::Vector3d a_enu{0, 0, 0};

  Nav_solution ns;
  Nav_correction co;
  Calib cal;
};
}
