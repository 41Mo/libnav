#include <iostream>
#include <fstream>
#include <string>
#include "calib.h"
#include "constants.h"

namespace NavA {
Calib::Calib(const char* name, calib_type t, char d) {
    std::ifstream file(name);
    if (!file.is_open()) {
        std::cout << "Error reading calib_file" << std::endl;
        return;
    }
    std::string coeff;
    std::getline(file, coeff); //skip first line
    switch (t) {
    case calib_type::invariant:
        // offsets vector
        for (uint8_t i=0; i<3; i++) {
            std::getline(file, coeff, d);
            B(i) = std::stod(coeff);
        }
        B = B*G;

        // scale factor diag
        for (uint8_t i=0; i<3; i++) {
            std::getline(file, coeff, d);
            A(i,i) = 1 + std::stod(coeff);
        }

        // non-linearity diag
        std::getline(file, coeff, d);
        A(1,0) = std::stod(coeff);
        std::getline(file, coeff, d);
        A(2,0) = std::stod(coeff);
        std::getline(file, coeff, d);
        A(2,1) = std::stod(coeff);
        A = matrix::inv(A);
        applied_cals[0] = invariant;
        break;
    
    default:
        break;
    }

}

void Calib::apply(D_IMU &imu) {
  auto a = imu.acc.value();
  for (auto &&i : applied_cals)
  {
      switch (i)
      {
      case invariant:
          a = A*(a-B);
          break;
      
      default:
          break;
      }
  }
};

}