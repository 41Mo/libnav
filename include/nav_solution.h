#pragma once
#include "math.hpp"

namespace NavA {
class Nav_solution {
 private:
  /*
      Orientation
  */

  /*
      Object rotation angles in body frame.
      0 element - pitch;
      1 element - roll;
      2 element - yaw;
  */
  matrix::Eulerd rotation;

  /*
      Navigation
  */

  /*
      Object velocity in ENU frame.
      0 element - V_e; east component
      1 element - V_n; north component
      2 element - V_up; vertical component
  */
  matrix::Vector3d velocity;

  /*
      Object coordinates in ENU frame.
      0 element - latitude;
      1 element - longtitude;
  */
  matrix::Vector2d position;

 public:
  /*
          Get velocity vector component by vos in vector.

  0 element - V_e; east component
  1 element - V_n; north component
  2 element - V_up; vertical component
  */
  const double& vel(size_t num);

  /*
          Get velocity vector.

  0 element - V_e; east component
  1 element - V_n; north component
  2 element - V_up; vertical component
  */
  const matrix::Vector3d& vel();

  /*
      Fill vec_in with vel components.

      0 element - V_e; east component
      1 element - V_n; north component
      2 element - V_up; vertical component
  */
  void vel(double vec_in[3]);

  /*
          Get rotation angle vector component by pos in vector.
  0 element - pitch;
  1 element - roll;
  2 element - yaw;
  */
  const double& rot(size_t num);

  /*
          Get rotation vector.

  0 element - pitch;
  1 element - roll;
  2 element - yaw;
  */
  const matrix::Eulerd& rot();

  /*
      Fill vec_in with rot components.

      0 element - pitch;
      1 element - roll;
      2 element - yaw;
  */
  void rot(double vec_in[3]);

  /*
          Get pos vector component by pos in vecotr.

  0 element - latitude;
  1 element - longtitude;
  */
  const double& pos(size_t num);

  /*
      Fill vec_in with pos components.

      0 element - latitude;
      1 element - longtitude;
  */
  void pos(double vec_in[2]);

  /*
          Get position vector.

  0 element - latitude;
  1 element - longtitude;
  */
  const matrix::Vector2d& pos();

 protected:
  friend class Nav;

  Nav_solution() {}
};
}