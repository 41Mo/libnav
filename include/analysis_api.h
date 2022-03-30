#ifndef analysis_api_h__
#define analysis_api_h__

#include "nav_alg.h"
#include "vectors.h"
#include "constants.h"

typedef struct
{
  int size;
  vec_body *acc;
  vec_body *gyr;
} SENSORS;

typedef struct
{
  float *roll;
  float *pitch;
  float *yaw;
  float *lat;
  float *lon;
  float *v_e;
  float *v_n;
} OUT;

#ifndef PYTHON
extern "C"
{
#endif
  class Analysis_api
  {
  private:
    int points;
    SENSORS sensors;
    OUT data;
    Nav nav{};

  public:
    Analysis_api();
    ~Analysis_api();
    void init(float lat, float lon, int time, int frequency);
    void loop();
    void set_sensors(SENSORS s);
    OUT get_data();

    // accessors to the nav alignment
    void alignment(float roll, float pitch, float yaw)
    {
      nav.alignment(roll, pitch, yaw);
    };
    void alignment(float ax_mean, float ay_mean, float az_mean, float yaw)
    {
      nav.alignment(ax_mean, ay_mean, az_mean, yaw);
    };
    void alignment(float st, float ct, float sg, float cg, float sp, float cp)
    {
      nav.alignment(st, ct, sg, cg, sp, cp);
    };

	  // special function just for analysis purposes
    void nav_get_prh(vec_body *v) { nav.get_prh(v); }
  };

#ifndef PYTHON
}
#endif

#ifdef PYTHON
extern "C"
{
  Analysis_api *Analysis_api_new() { return new Analysis_api(); }
  void api_init(Analysis_api *api, float lat,
                float lon, int frequency, int time)
  {
    api->init(lat, lon, frequency, time);
  }
  void api_loop(Analysis_api *api) { api->loop(); }
  void api_set_sens(Analysis_api *api, SENSORS s) { api->set_sensors(s); }
  OUT api_get_data(Analysis_api *api) { return api->get_data(); }
  float api_get_u(void) { return U; }
  float api_get_g(void) { return G; }

  void api_alignment_rph(Analysis_api *api, float roll, float pitch, float yaw)
  {
    api->alignment(roll, pitch, yaw);
  };
  void api_alignment_acc(Analysis_api *api, float ax_mean, float ay_mean, float az_mean, float yaw)
  {
    api->alignment(ax_mean, ay_mean, az_mean, yaw);
  };
  void api_alignment_cos(Analysis_api *api, float st, float ct, float sg, float cg, float sp, float cp)
  {
    api->alignment(st, ct, sg, cg, sp, cp);
  };

  void api_get_prh(Analysis_api *api, vec_body *v) { api->nav_get_prh(v); }
}
#endif
#endif
