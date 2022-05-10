#ifndef nav_cor_h_
#define nav_cor_h_

class Nav_correction {

public:
 
 void on_off_gnss_corr(bool Mode);
 void set_time_gnss_corr(float Time);
 void corr_coef(float dphi, float dlam);

private:

matrix::Vector2f dpos;
float w_s{0.0f};
float T;
float gnss_coeff[3]{0,0,0};

protected:
  friend class Nav;

  Nav_correction() {}

};

#endif // nav_cor_h_
