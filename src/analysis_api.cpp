#include "analysis_api.h"

#include "stdio.h"
#include "vectors.h"

Analysis_api::Analysis_api() {}

Analysis_api::~Analysis_api() {}

void init_array(float arr[], int size) {
  for (auto i = 0; i< size; i++) {
    arr[i] = 0;
  }
}

void Analysis_api::init(float lat, float lon, int time, int frequency) {
  points = time * frequency;
  nav = std::unique_ptr<Nav>(new Nav(lat, lon, frequency));
  // init arrays
  data.roll = new float[points];
  data.pitch = new float[points];
  data.heading = new float[points];
  data.lat = new float[points];
  data.lon = new float[points];
  data.v_e = new float[points];
  data.v_n = new float[points];
  init_array(data.roll, points);
  init_array(data.pitch, points);
  init_array(data.heading, points);
  init_array(data.lat, points);
  init_array(data.lon, points);
  init_array(data.v_e, points);
  init_array(data.v_n, points);
}

void Analysis_api::loop() {
  if (sensors.gyr == nullptr || sensors.acc == nullptr) {
    printf ("data error");
    return;
  }
  for (int i = 0; i < points; i++) {
    nav->iter(sensors.acc[i], sensors.gyr[i]);

    // add data to array on each iteration
    data.roll[i]  = nav->sol().rot(1) ;
    data.pitch[i] = nav->sol().rot(0);
    if (nav->sol().rot(2) < 0) {
      data.heading[i] = nav->sol().rot(2);//+3.14*2;
    } else {
      data.heading[i] = nav->sol().rot(2);
    }
    data.lat[i]   = nav->sol().pos(0);
    data.lon[i]   = nav->sol().pos(1);
    data.v_e[i]   = nav->sol().vel(0);
    data.v_n[i]   = nav->sol().vel(1);
  }
}

void Analysis_api::set_sensors(SENSORS s) {
  sensors.acc = s.acc;
  sensors.gyr = s.gyr;
  sensors.size = s.size;
}

OUT Analysis_api::get_data() {
  return data;
}
