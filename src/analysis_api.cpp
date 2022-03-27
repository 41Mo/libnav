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

void Analysis_api::init(float roll, float pitch, float yaw, float lat, float lon, int time, int frequency) {
  points = time * frequency;
  nav.init(lat, lon, frequency);
  nav.alignment(roll, pitch, yaw);

  // init arrays
  //data.size = points;
  data.roll = new float[points];
  data.pitch = new float[points];
  data.yaw = new float[points];
  data.lat = new float[points];
  data.lon = new float[points];
  data.v_e = new float[points];
  data.v_n = new float[points];
  init_array(data.roll, points);
  init_array(data.pitch, points);
  init_array(data.yaw, points);
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
    nav.iter(sensors.acc[i], sensors.gyr[i]);

    // add data to array on each iteration
    data.roll[i] = nav.gamma;
    data.pitch[i] = nav.teta;
    data.yaw[i] = nav.psi;
    data.lat[i] = nav.phi;
    data.lon[i] = nav.lambda;
    data.v_e[i] = nav.v_enu.E;
    data.v_n[i] = nav.v_enu.N;
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
