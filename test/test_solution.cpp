#include <iostream>
#include <math.hpp>
#include "interface.h"
#include "test_macros.hpp"
static const float ANGLE_EPS = 0.007*M_PI/180;
static const float VEL_EPS = 4;
static const float POS_EPS = 0.06 * M_PI /180;
void trace_print(size_t i, size_t j, float eq_elem, float alg_elem, float a_delta){
    std::cout << "Iteration: " << i << std::endl;
    std::cout << "Elem num: " << j << std::endl;
    std::cout << "EQ elem: " << eq_elem << std::endl;
    std::cout << "Alg elem: " << alg_elem << std::endl;
    std::cout << "Assumed delta: " << a_delta << std::endl;
    std::cout << "Delta: " << fabs(eq_elem - alg_elem) << std::endl;
}

int main(int argc, char const *argv[])
{
    int lat = 0;
    int lon = 0;

    uint32_t sample_time = 5400;
    float data_frequency = 10;

    auto pry = matrix::Eulerf(0, 0, 0);

    matrix::Vector3f gyro_drift = {1*M_PI/180/3600, 2*M_PI/180/3600, 0};
    matrix::Vector3f acc_offset = {0.001f * 9.8f, 0.002f * 9.8f, 0};

    matrix::Vector3f a_enu(0, 0, 9.8);
    matrix::Vector3f w_enu(0, U*cosf32(lat), U*sinf32(lat));

    auto C_enu_body = matrix::Dcmf(pry);

    auto a_body = C_enu_body * a_enu;
    auto w_body = C_enu_body * w_enu;

    TEST(isEqual(a_body, a_enu));
    TEST(isEqual(w_body, w_enu));

    size_t points = sample_time*data_frequency;
    auto iface = NavIface(lat, lon, data_frequency);

    float abx = a_body(0,0) + acc_offset(0);
    float aby = a_body(1,0) + acc_offset(1);
    float abz = a_body(2,0) + acc_offset(2);

    iface.nav()->alignment(abx, aby, abz, pry(2));
    float align_pry[3];
    iface.nav()->get_prh(align_pry);
    auto r = matrix::Vector2f(align_pry);
    r(0) -= pry(0);
    r(1) -= pry(2);

    auto eq_pry_err = matrix::Vector2f(
        acc_offset(1) / sqrtf32(powf32(G,2) - powf32(aby,2)),
        -(acc_offset(0) * abz) / (pow(abx,2) + pow(abz,2))
    );
    TEST(isEqual(eq_pry_err, r)); 
    
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
            deltaPhi, deltaLambda
        );
        for (size_t j = 0; j < 2; j++)
        {
            if (!(fabs(-alg_pry(j) + eq_pry(j)) < ANGLE_EPS)) {
                trace_print(i, j, eq_pry(j), alg_pry(j), ANGLE_EPS);
                TEST(0);
            }
            if (!(fabs(-alg_vel(j)+eq_vel(j)) < VEL_EPS)) {
                trace_print(i, j, eq_vel(j), alg_vel(j), VEL_EPS);
                TEST(0);
            }
            if(!(fabs(-alg_pos(j)+eq_pos(j)) < POS_EPS)) {
                trace_print(i, j, eq_pos(j), alg_pos(j), POS_EPS);
                TEST(0);
            }
        }
    }
    
    return 0;
}
