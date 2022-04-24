#include <math.h>
#include <stdio.h>

#include "nav_alg.h"
#include "constants.h"

Nav::Nav(float p, float l, int f)
{
	coord(0) = p;
	coord(1) = l;
	frequency = f;
	dt = 1/float(frequency);
	ns.setup(&v_enu, &pry, &coord);
}

Nav::Nav(int f)
{
	frequency = f;
	dt = 1/float(frequency);
	ns.setup(&v_enu, &pry, &coord);
}

void Nav::set_pos(float p, float l) {
	coord(0) = p;
	coord(1) = l;
}

void Nav::puasson_equation(matrix::Vector3f &w_body) 
{
	dcm = dcm + (dcm * w_body.hat() - w_enu.hat() * dcm ) * dt;
}

void Nav::euler_angles()
{
	pry = matrix::Eulerf(dcm);
}

void Nav::get_prh(vec_body *v)
{
	float c0 = sqrt(pow(dcm(2,0), 2) + pow(dcm(2,2), 2));
	float teta = atan(dcm(2,1)/c0);
	float gamma = -atan(dcm(2,0)/dcm(2,2));
	float psi = atan2f(dcm(0,1), dcm(1,1));

	//printf("Old:\nteta:%f; gamma:%f; psi:%f\n", teta, gamma, psi);
	//auto test = matrix::Eulerf(dcm);
	//printf("New:\nteta:%f; gamma:%f; psi:%f\n", test(0), test(1), test(2));

	v->X = teta;
	v->Y = gamma;
	v->Z = psi;
}

void Nav::acc_body_enu(matrix::Vector3f &a_body)
{
	a_enu = dcm * a_body;
}

void Nav::speed()
{
	v_enu(0) = v_enu(0) + (a_enu(0) + (U * sin(coord(0)) + w_enu(2)) * v_enu(1)) * dt;
	v_enu(1) = v_enu(1) + (a_enu(1) - (U * sin(coord(0)) + w_enu(2)) * v_enu(0)) * dt;

}

void Nav::coordinates()
{
	// Latitude
	coord(0) = coord(0) + (v_enu(1) / (R + H)) * dt;
	// Longitude
	coord(1) = coord(1) + (v_enu(0) / ((R + H) * cos(coord(0)))) * dt;
}

void Nav::ang_velocity_body_enu()
{
	w_enu(0) = -v_enu(1) / (R + H);
	w_enu(1) = v_enu(0) / (R + H) + U * cos(coord(0));
	w_enu(2) = (v_enu(0) / (R + H)) * tan(coord(0)) + U * sin(coord(0));
}

void Nav::alignment(float roll, float pitch, float yaw)
{
	float psi = yaw;
	float teta = pitch;
	float gamma = roll;
	
	float sp = sin(psi); float st = sin(teta); float sg = sin(gamma);
	
	float cp = cos(psi); float ct = cos(teta); float cg = cos(gamma);

	alignment(st, ct, sg, cg, sp, cp);
}

void Nav::alignment(float st, float ct, float sg, float cg, float sp, float cp) {
	dcm(0,0) = cp * cg + sp * st * sg;
	dcm(0,1) = sp * ct;
	dcm(0,2) = cp * sg - sp * st * cg;
	dcm(1,0) = - sp * cg + cp * st * sg;
	dcm(1,1) = cp * ct;
	dcm(1,2) = - sp * sg - cp * st * cg;
	dcm(2,0) = - ct * sg;
	dcm(2,1) = st;
	dcm(2,2) = ct * cg;
}

void Nav::alignment(float ax, float ay, float az, float yaw) {
	float psi = yaw;
	
	float A = sqrt(pow(ax, 2) + pow(az, 2));

	float st = ay/G;
	float sg = -1 * ax/A;
	
	float ct = A/G;
	float cg = az/A;

	float sp = sin(psi);
	float cp = cos(psi);

	alignment(st, ct, sg, cg, sp, cp);
}

/*
void Nav::norm_row() 
{
	float c1i = pow(c11,2) + pow(c12,2) + pow(c13,2);
	float c2i = pow(c21,2) + pow(c22,2) + pow(c23,2);
	float c3i = pow(c31,2) + pow(c32,2) + pow(c33,2);

	if (c1i > 1) c1i = 1;
	if (c2i > 1) c2i = 1;
	if (c3i > 1) c3i = 1;

	c11 = c11 / c1i;
	c12 = c12 / c1i;
	c13 = c13 / c1i;
	c21 = c21 / c2i;
	c22 = c22 / c2i;
	c23 = c23 / c2i;
	c31 = c31 / c3i;
	c32 = c32 / c3i;
	c33 = c33 / c3i;
}

void Nav::norm_column() 
{
	float cj1 = pow(c11,2) + pow(c21,2) + pow(c31,2);
	float cj2 = pow(c12,2) + pow(c22,2) + pow(c32,2);
	float cj3 = pow(c13,2) + pow(c23,2) + pow(c33,2);

	if (cj1 > 1) cj1 = 1;
	if (cj2 > 1) cj2 = 1;
	if (cj3 > 1) cj3 = 1;

	c11 = c11 / cj1;
	c12 = c12 / cj2;
	c13 = c13 / cj3;
	c21 = c21 / cj1;
	c22 = c22 / cj2;
	c23 = c23 / cj3;
	c31 = c31 / cj1;
	c32 = c32 / cj2;
	c33 = c33 / cj3;
}

void Nav::normalization()
{
	if (i <= 5) {
		norm_row();
		i++;
	} else if (i <= 10) {
		norm_column();
		i++;
	} else {
		i = 0;
	}
}
*/

void Nav::iter(const vec_body& acc, const vec_body& gyr)
{
	auto a = matrix::Vector3f(acc.X, acc.Y, acc.Z);
	auto g = matrix::Vector3f(gyr.X, gyr.Y, gyr.Z);
	acc_body_enu(a);
	ang_velocity_body_enu();
	puasson_equation(g);
	speed();
	euler_angles();
	coordinates();
}

void Nav::iter(matrix::Vector3f &acc, matrix::Vector3f &gyr)
{
	acc_body_enu(acc);
	ang_velocity_body_enu();
	puasson_equation(gyr);
	speed();
	euler_angles();
	coordinates();
}

void Nav::iter(float acc[3], float gyr[3])
{
	auto a = matrix::Vector3f(acc);
	auto g = matrix::Vector3f(gyr);
	acc_body_enu(a);
	ang_velocity_body_enu();
	puasson_equation(g);
	speed();
	euler_angles();
	coordinates();
}

/* TODO:
	add a function to convert the magnetometer
	yaw from body to enu
vec_enu Nav::mag_to_enu()
*/
