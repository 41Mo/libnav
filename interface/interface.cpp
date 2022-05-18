#include <iostream>

#include "interface.h"

NavIface::NavIface(float lat, float lon, int frequency) {
  if (frequency <= 0) {
    nav_alg = Nav(lat,lon, 10);
  } else {
    nav_alg = Nav(lat, lon, frequency);
  }
}

NavIface::~NavIface() {}