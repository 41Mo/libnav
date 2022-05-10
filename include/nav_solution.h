#ifndef Pose_h__
#define Pose_h__

#include "math.hpp"

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
  matrix::Eulerf rotation;

  /*
      Navigation
  */

  /*
      Object velocity in ENU frame.
      0 element - V_e; east component
      1 element - V_n; north component
      2 element - V_up; vertical component
  */
  matrix::Vector3f velocity;

  /*
      Object coordinates in ENU frame.
      0 element - latitude;
      1 element - longtitude;
  */
  matrix::Vector2f position;

 public:
  /*
          Get velocity vector component by vos in vector.

  0 element - V_e; east component
  1 element - V_n; north component
  2 element - V_up; vertical component
  */
  const float& vel(size_t num);

  /*
          Get velocity vector.

  0 element - V_e; east component
  1 element - V_n; north component
  2 element - V_up; vertical component
  */
  const matrix::Vector3f& vel();

  /*
      Fill vec_in with vel components.

      0 element - V_e; east component
      1 element - V_n; north component
      2 element - V_up; vertical component
  */
  void vel(float vec_in[3]);

  /*
          Get rotation angle vector component by pos in vector.
  0 element - pitch;
  1 element - roll;
  2 element - yaw;
  */
  const float& rot(size_t num);

  /*
          Get rotation vector.

  0 element - pitch;
  1 element - roll;
  2 element - yaw;
  */
  const matrix::Eulerf& rot();

  /*
      Fill vec_in with rot components.

      0 element - pitch;
      1 element - roll;
      2 element - yaw;
  */
  void rot(float vec_in[3]);

  /*
          Get pos vector component by pos in vecotr.

  0 element - latitude;
  1 element - longtitude;
  */
  const float& pos(size_t num);

  /*
      Fill vec_in with pos components.

      0 element - latitude;
      1 element - longtitude;
  */
  void pos(float vec_in[2]);

  /*
          Get position vector.

  0 element - latitude;
  1 element - longtitude;
  */
  const matrix::Vector2f& pos();

 protected:
  friend class Nav;

  Nav_solution() {}
};
#endif  // Pose_h__
