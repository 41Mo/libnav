#include <stdint.h>
#include <math.h>
#include "vectors.h"

class Nav {
public:
    Nav();
    ~Nav() {};

	/*
		Algorithm initialization based on current coordinates and frequency.
	*/
	void init(float phi, float lambda, int frequency);

	/*
		Alignment based on 3 angles of rotation 
	*/
	void alignment(float roll, float pitch, float yaw);

	/*
		Alignment based on mean values of acceleration along 3 axis in body frame and yaw.
	*/
	void alignment(float ax_mean, float ay_mean, float az_mean, float yaw);

	/*
		Alignment based on sin and cos of 3 angles of rotation.
	*/
	void alignment(float st, float ct, float sg, float cg, float sp, float cp);
	void iter(vec_body acc, vec_body gyr);
	
	/* 
	Special function just for analysis purposes.
	Primarly used to check if alignment was successfull.
	*/
	void get_prh(vec_body *v);



private:
	void puasson_equation();
	void euler_angles();
	void acc_body_enu();
	void speed();
	void coordinates();
	void ang_velocity_body_enu();
	void norm_row();
	void norm_column();
	void normalization();

	/*
		All variables are represented in SI.
	*/

	/*
		Direction cosine matrix initial values
	*/
	float c11{1}, c12{0}, c13{0}, c21{0}, c22{1}, c23{0}, c31{0}, c32{0}, c33{1};

	float H{0};	// height above ground
	float teta{0}; // pitch
	float gamma{0}; // roll
	float psi{0}; // yaw
	float phi{0}; // latitude
	float lambda{0}; // lontitude
	int frequency{0};
	float dt{0};
	int i{0};

	vec_body w_body {0, 0, 0};
	vec_body a_body {0, 0, 0};
	vec_enu v_enu {0, 0, 0};
	vec_enu w_enu{0,0,0};
	vec_enu a_enu{0,0,0};
};

/*
// Convert the angle given in radians to degrees.
template<typename F>
F rad2deg(F angle) {
    return angle * 180.0 / M_PI;
}

// Convert the angle given in radians to degrees.
template<typename F>
F deg2rad(F angle) {
    return angle * M_PI / 180.0;
}
*/
