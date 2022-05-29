#include "interface.h"
#include "nav_alg.h"
NavIface::NavIface(float lat, float lon, int frequency) {
  using namespace NavA;
  if (frequency <= 0) {
    nav_alg = Nav(lat,lon, 10);
  } else {
    nav_alg = Nav(lat, lon, frequency);
  }
}

NavIface::~NavIface() {}