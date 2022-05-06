#ifndef nav_cor_h_
#define nav_cor_h_
class Nav_correction {
private:

  /*
      Navigation
  */

  /*
      Object velocity SNS in ENU frame.
      0 element - V_e; east component
      1 element - V_n; north component
      2 element - V_up; vertical component
  */
  matrix::Vector3f velocity_sns;

  /*
      Object coordinates SNS in ENU frame.
      0 element - latitude;
      1 element - longtitude;
  */
  matrix::Vector2f position_sns;

 public:
  /*
          Get velocity SNS vector component by vos in vector.

  0 element - V_e; east component
  1 element - V_n; north component
  2 element - V_up; vertical component
  */
  const float& vel_sns(size_t num);

  /*
          Get velocity SNS vector.

  0 element - V_e; east component
  1 element - V_n; north component
  2 element - V_up; vertical component
  */
  const matrix::Vector3f& vel_sns();

  /*
      Fill vec_in with vel SNS components.

      0 element - V_e; east component
      1 element - V_n; north component
      2 element - V_up; vertical component
  */
  void vel_sns(float vec_in[3]);

  /*
          Get pos SNS vector component by pos in vecotr.

  0 element - latitude;
  1 element - longtitude;
  */
  const float& pos_sns(size_t num);

  /*
      Fill vec_in with pos SNS components.

      0 element - latitude;
      1 element - longtitude;
  */
  void pos_sns(float vec_in[2]);

  /*
          Get position SNS vector.

  0 element - latitude;
  1 element - longtitude;
  */
  const matrix::Vector2f& pos_sns();
  
public:
 void on_off_corr(bool Mode);
 void set_time_corr(uint32_t Time);

private:

float k1{0.0f};
float k2{0.0f};
float k3{0.0f};
float w_s{0.0f};
float T;

protected:
  friend class Nav;

  Nav_correction() {}

};

#endif // nav_cor_h_
