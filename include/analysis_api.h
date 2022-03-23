#ifndef analysis_api_h__
#define analysis_api_h__

#include "nav_alg.h"
#include "vectors.h"
#include "constants.h"

typedef struct {
  int size;
  vec_body *acc;
  vec_body *gyr;
} SENSORS;

typedef struct {
  float *roll;
  float *pitch;
  float *yaw;
  float *lat;
  float *lon;
  float *v_e;
  float *v_n;
} OUT;

#ifndef PYTHON
extern "C" {
#endif
class Analysis_api {
 private:
  /* data */
  int points;
  SENSORS sensors;
  OUT data;
  Nav nav{};

 public:
  Analysis_api();
  ~Analysis_api();
  void init(float roll, float pitch, float yaw, float lat, float lon, int time,
            int frequency);
  void loop();
  void set_sensors(SENSORS s);
  OUT get_data();
};

#ifndef PYTHON
}
#endif

#ifdef PYTHON
extern "C" {
  Analysis_api *Analysis_api_new() { return new Analysis_api(); }
  void api_init(Analysis_api *api, float roll, float pitch, float yaw, float lat,
                float lon, int frequency, int time) {
    api->init(roll, pitch, yaw, lat, lon, frequency, time);
  }
  void api_loop(Analysis_api *api) { api->loop(); }
  void api_set_sens(Analysis_api *api, SENSORS s) { api->set_sensors(s); }
  OUT api_get_data(Analysis_api *api) { return api->get_data(); }
  float api_get_u(void) { return U; }
  float api_get_g(void) { return G; }
}
#endif
#endif
