#include <iostream>
#include <math.hpp>
#include "interface.h"
#include "test_macros.hpp"
#define ANGLE_EPS 1*M_PI/180
#define VEL_EPS 20
#define POS_EPS 10 * M_PI /180
void print_vec_body(matrix::Matrix<float, 3UL, 1UL> m) {
    for (size_t i = 0; i < 3; i++)
    {
        std::cout << m(i,0) << " ";
    }
}

int main(int argc, char const *argv[])
{
    int lat = 0;
    int lon = 0;

    uint32_t sample_time = 10800;
    float data_frequency = 100;

    auto pry = matrix::Eulerf(0, 0, 0);

    matrix::Vector3f gyro_drift = {1*M_PI/180/3600, 1*M_PI/180/3600, 0};
    matrix::Vector3f acc_offset = {0.002f * 9.8f, 0.002f * 9.8f, 0};

    matrix::Vector3f a_enu(0, 0, 9.8);
    matrix::Vector3f w_enu(0, U*cosf32(lat), U*sinf32(lat));

    auto C_enu_body = matrix::Dcmf(pry);

    auto a_body = C_enu_body * a_enu;
    auto w_body = C_enu_body * w_enu;

    TEST(isEqual(a_body, a_enu));
    TEST(isEqual(w_body, w_enu));

    size_t points = sample_time*data_frequency;
    auto iface = NavIface(lat, lon, data_frequency);

    iface.nav()->alignment(a_body(0,0), a_body(1,0), a_body(2,0), pry(2));
    float align_pry[3];
    iface.nav()->get_prh(align_pry);

    auto r = matrix::Vector3f(align_pry);
    TEST(isEqual(r, pry)); 
    
    auto nu = sqrtf32(G/R);
    auto gd_enu = C_enu_body.transpose() * gyro_drift;
    auto ao_enu = C_enu_body.transpose() * acc_offset;
    auto dt = 1/data_frequency;
    float t = 0;

    for (size_t i = 0; i < points; i++)
    {
        // nav alg error equations
        auto Phi_ox = -ao_enu(1,0)/G - gd_enu(0,0)*sinf32(nu*t)/nu;
        auto Phi_oy = ao_enu(0,0)/G - gd_enu(1,0)*sinf32(nu*t)/nu;
        auto deltaVx = gd_enu(1,0)*R*(1-cosf32(nu*t));
        auto deltaVy = -gd_enu(0,0)*R*(1-cosf32(nu*t));
        auto deltaLambda = gd_enu(1,0)*(t - sinf32(nu*t)/nu);
        auto deltaPhi = -gd_enu(0,0)*(t - sinf32(nu*t)/nu);
        auto pitch = -(Phi_ox*cosf32(pry(2)) - Phi_oy*sinf32(pry(2)));
        auto roll = -(Phi_oy*cosf32(pry(2)) + Phi_ox*sinf32(pry(2)))*1/cosf32(pry(0));
        t+=dt;

        // nav alg data
        float a_sens[3]; float g_sens[3];
        for (size_t j = 0; j < 3; j++)
        {
            a_sens[j] = a_body(j,0) + acc_offset(j);
            g_sens[j] = w_body(j,0) + gyro_drift(j);
        }
        iface.nav()->iter(a_sens, g_sens);

        auto alg_pry = matrix::Vector2f(
            iface.nav()->sol().rot()
        );
        auto eq_pry = matrix::Vector2f(
            pitch, roll
        );
        auto alg_vel = matrix::Vector2f(
            iface.nav()->sol().vel()
        );
        auto eq_vel = matrix::Vector2f(
            deltaVx, deltaVy
        );
        auto alg_pos = iface.nav()->sol().pos();
        auto eq_pos = matrix::Vector2f(
            deltaLambda, deltaPhi
        );
        for (size_t j = 0; j < 2; j++)
        {
            TEST((fabs(alg_pry(j) - eq_pry(j)) < ANGLE_EPS));
            TEST((fabs(alg_vel(j)-eq_vel(j)) < VEL_EPS));
            TEST((fabs(alg_pos(j)-eq_pos(j)) < POS_EPS));
        }
    }
    
    return 0;
}
