#pragma once
#include "math.hpp"
#include "input_structure.h"

namespace NavA {
class Calib
{
public:
    enum calib_type {
        none,
        invariant,
        temperature
    };
    Calib(){};
    Calib(
        const char* calib_file,
        calib_type t,
        char delimeter = ',');
    ~Calib(){};
    void apply(D_IMU &imu);

private:
    matrix::Matrix3d A;
    matrix::Vector3d B;
    calib_type applied_cals[2] {none,none};
};
}