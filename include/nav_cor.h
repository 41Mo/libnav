#ifndef nav_cor_h_
#define nav_cor_h_
class Nav_correction {
public:
 void on_off_corr(bool Mode);
 void set_time_corr(uint32_t Time);

private:

float k1{0};
float k2{0};
float k3{0};
float w_s{0};
uint32_t T;

protected:
  friend class Nav;

  Nav_correction() {}

};

#endif // nav_cor_h_
