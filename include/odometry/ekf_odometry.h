#pragma once

#include "types/structs.h"

struct ekf_process_noise_t {
    double pos_xy = 5.0;
    double theta = 5.0;
    double vel_xy = 50.0;
    double omega = 30.0;
};

struct ekf_measurement_noise_t {
    double otos_vx = 120.0;
    double otos_vy = 120.0;
    double imu_omega = 25.0;
};

struct ekf_debug_t {
    double innovation[5];
    double innovation_sigma[5];
};

class OdometryEkf
{
  public:
    OdometryEkf();
    void reset(const position_t &pose);
    void setProcessNoise(const ekf_process_noise_t &noise);
    void setMeasurementNoise(const ekf_measurement_noise_t &noise);
    void setGateSigma(double sigma);

    void predict(double dt, double gyro_omega);
    void update(const double z[5], const double r_left[2], const double r_right[2], const bool valid[5], ekf_debug_t &dbg);

    void getState(position_t &pos, position_t &vel) const;
    void getCovarianceDiag(double diag[6]) const;

  private:
    double _x[6];
    double _p[6][6];
    ekf_process_noise_t _q;
    ekf_measurement_noise_t _r;
    double _gate_sigma;
};
