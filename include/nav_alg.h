#include <stdint.h>
#include <math.h>
#include "vectors.h"

class Nav {
public:
    // constructor
    Nav();
    // destructor
    ~Nav() {};

	void puasson_equation();
	void euler_angles();
	void acc_body_enu();
	void speed();
	void coordinates();
	void ang_velocity_body_enu();
	void alignment(float roll, float pitch, float yaw);
	void alignment(float ax_mean, float ay_mean, float az_mean, float yaw);
	void alignment(float st, float ct, float sg, float cg, float sp, float cp);
	void iter(vec_body acc, vec_body gyr);
	void init(float phi, float lambda, int frequency);
	
	/* 
	Special function just for analysis purposes.
	Primarly used to check if alignment was successfull.
	*/
	void get_prh(vec_body *v);

	// variables
	float c11{0}, c12{0}, c13{0}, c21{0}, c22{0}, c23{0}, c31{0}, c32{0}, c33{0}; //начальные элементы матрицы

	float H{0};
	float teta{0};
	float gamma{0};
	float psi{0};
	float phi{0};
	float lambda{0};
	int frequency{0};
	float dt{0};

	vec_body w_body {0, 0, 0};
	vec_body a_body {0, 0, 0};
	vec_enu v_enu {0, 0, 0};
	vec_enu w_enu{0,0,0};
	vec_enu a_enu{0,0,0};


private:
};


/**
 * Convert the angle given in radians to degrees.
 */
template<typename F>
F rad2deg(F angle) {
    return angle * 180.0 / M_PI;
}

/**
 * Convert the angle given in radians to degrees.
 */
template<typename F>
F deg2rad(F angle) {
    return angle * M_PI / 180.0;
}
