#include <math.h>
#include "nav_alg.h"
#include <stdio.h>
#include "constants.h"

Nav::Nav()
{
}

void Nav::init(float p, float l, int f) {
	phi = p;
	lambda = l;
	frequency = f;
	Nav::dt = 1/float(frequency);
}

void Nav::puasson_equation() 
{
	c11 = c11 - dt * (c13 * w_body.Y + c31 * w_enu.N - c12 * w_body.Z - c21 * w_enu.U);
	c12 = c12 + dt * (c13 * w_body.X - c32 * w_enu.N - c11 * w_body.Z + c22 * w_enu.U);
	c13 = c13 - dt * (c12 * w_body.X - c11 * w_body.Y + c33 * w_enu.N - c23 * w_enu.U);
	c21 = c21 + dt * (c31 * w_enu.E - c23 * w_body.Y + c22 * w_body.Z - c11 * w_enu.U);
	c22 = c22 + dt * (c23 * w_body.X + c32 * w_enu.E - c21 * w_body.Z - c12 * w_enu.U);
	c23 = c23 - dt * (c22 * w_body.X - c33 * w_enu.E - c21 * w_body.Y + c13 * w_enu.U);
	c31 = c12 * c23 - c13 * c22;
	c32 = - (c11 * c23 - c13 * c21);
	c33 = c11 * c22 - c12 * c21;
}

void Nav::euler_angles()
{
	float c0 = sqrt(pow(c31, 2) + pow(c33, 2));
	teta = atan(c32/c0);
	gamma = -atan(c31/c33);
	psi = atan2f(c12, c22);
}

void Nav::get_prh(vec_body *v)
{
	float c0 = sqrt(pow(c31, 2) + pow(c33, 2));
	teta = atan(c32/c0);
	gamma = -atan(c31/c33);
	psi = atan2f(c12, c22);

	v->X = teta;
	v->Y = gamma;
	v->Z = psi;
}

void Nav::acc_body_enu()
{
	a_enu.E = c11 * a_body.X + c12 * a_body.Y + c13 * a_body.Z;
	a_enu.N = c21 * a_body.X + c22 * a_body.Y + c23 * a_body.Z;
	a_enu.U = c31 * a_body.X + c32 * a_body.Y + c33 * a_body.Z;
}

void Nav::speed()
{
	v_enu.E = v_enu.E + (a_enu.E + (U * sin(phi) + w_enu.U) * v_enu.N) * dt;
	v_enu.N = v_enu.N + (a_enu.N - (U * sin(phi) + w_enu.U) * v_enu.E) * dt;

}

void Nav::coordinates()
{
	// Latitude
	phi = phi + (v_enu.N / (R + H)) * dt;
	// Longitude
	lambda = lambda + (v_enu.E / ((R + H) * cos(phi))) * dt;
}

void Nav::ang_velocity_body_enu()
{
	w_enu.E = -v_enu.N / (R + H);
	w_enu.N = v_enu.E / (R + H) + U * cos(phi);
	w_enu.U = (v_enu.E / (R + H)) * tan(phi) + U * sin(phi);
}

void Nav::alignment(float roll, float pitch, float yaw)
{
	psi = yaw;
	teta = pitch;
	gamma = roll;
	
	float sp = sin(psi); float st = sin(teta); float sg = sin(gamma);
	
	float cp = cos(psi); float ct = cos(teta); float cg = cos(gamma);

	alignment(st, ct, sg, cg, sp, cp);
}

void Nav::alignment(float st, float ct, float sg, float cg, float sp, float cp) {
	c11 = cp * cg + sp * st * sg;
	c12 = sp * ct;
	c13 = cp * sg - sp * st * cg;
	c21 = - sp * cg + cp * st * sg;
	c22 = cp * ct;
	c23 = - sp * sg - cp * st * cg;
	c31 = - ct * sg;
	c32 = st;
	c33 = ct * cg;
}

void Nav::alignment(float ax, float ay, float az, float yaw) {
	psi = yaw;
	
	float A = sqrt(pow(ax, 2) + pow(az, 2));

	float st = ay/G;
	float sg = -1 * ax/A;
	
	float ct = A/G;
	float cg = az/A;

	float sp = sin(psi);
	float cp = cos(psi);

	alignment(st, ct, sg, cg, sp, cp);
}
//vec_enu Nav::mag_to_enu()

void Nav::iter(vec_body acc, vec_body gyr)
{
	w_body = gyr;
	a_body = acc;
	acc_body_enu();
	ang_velocity_body_enu();
	puasson_equation();
	speed();
	euler_angles();
	coordinates();
}
