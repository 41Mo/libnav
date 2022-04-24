#include <stdint.h>
#include <math.h>
#include "math.hpp"
#include "vectors.h"
#include "nav_solution.h"
class Nav {
public:

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
		Alignment based on mean values of acceleration along 3 axis in body frame and yaw.
	*/
	void alignment(float ax_mean, float ay_mean, float az_mean, float yaw);

	/*
		Alignment based on sin and cos of 3 angles of rotation.
	*/
	void alignment(float st, float ct, float sg, float cg, float sp, float cp);

	/*
		Do 1 iteration over acc and gyr data.
	*/
	void iter(matrix::Vector3f &acc, matrix::Vector3f &gyr);
	void iter(float acc[3], float gyr[3]);
	void iter(const vec_body& a, const vec_body& g);

	/*
		Get solution for current iteration.
	*/
	Nav_solution& sol() { return ns; }
	/* 
	Special function just for analysis purposes.
	Primarly used to check if alignment was successfull.
	*/
	void get_prh(vec_body *v);

private:
	void puasson_equation(matrix::Vector3f &w_body);
	void euler_angles();
	void acc_body_enu(matrix::Vector3f &a_body);
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
		Direction cosine matrix
	*/
	matrix::Dcmf dcm;

	/*
		Euler angles
	*/
	matrix::Eulerf pry;

	/*
		Holding current coordinates
		0 elem - latitide
		1 elem - longtitude
	*/
	matrix::Vector2f coord;

	float H{0};	// height above ground
	//float phi{0}; // latitude
	//float lambda{0}; // lontitude
	int frequency{0}; // operation frequency
	float dt{0};
	int i{0};

	matrix::Vector3f v_enu{0,0,0};
	matrix::Vector3f w_enu{0,0,0};
	matrix::Vector3f a_enu{0,0,0};

	Nav_solution ns;

/*
	Remove default constructor.
	Frequency is mandatory for Nav to operate.
*/
public: Nav() = delete;

/*
	Remove copy constructor.
*/
public: Nav (const Nav&) = delete;
public: Nav& operator= (const Nav&) = delete;

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
