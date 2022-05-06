#include <iostream>

#include "interface.h"

NavIface::NavIface(float lat, float lon, int frequency) {
  nav_alg = Nav(lat, lon, frequency);
}

NavIface::~NavIface() {}

NAV_OUT NavIface::solution(SENS_IN s) {
  if (!s.size) {
    std::cout << "Sensor data cannot be empty" << std::endl;
    exit(-1);
  }

  if (s.a == nullptr || s.g == nullptr) {
    std::cout << "Input data error" << std::endl;
    exit(-1);
  }

  size_t points = s.size;

  Vector3f pry[points];
  Vector2f coord[points];
  Vector2f vel[points];
  Vector2f vel_sns[points];

  for (size_t i = 0; i < points; i++) {
    nav_alg.iter(s.a[i].data, s.g[i].data);

    nav_alg.sol().rot(pry[i].data);
    nav_alg.sol().pos(coord[i].data);
    nav_alg.sol().vel(vel[i].data);
    nav_alg.sol().vel_sns(vel_sns[i].data);
  }
  NAV_OUT r{points, pry, coord, vel, vel_sns};
  return r;
}