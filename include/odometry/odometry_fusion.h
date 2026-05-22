#pragma once

#include "types/structs.h"
#include "odometry/dual_otos.h"
#include "odometry/ekf_odometry.h"

struct odometry_config_t {
    float sensor_spacing_mm = 120.0f;
    float lowpass_alpha = 0.6f;
    double gate_sigma = 3.5;
    uint8_t address_left = kDefaultAddress;
    uint8_t address_right = (uint8_t)(kDefaultAddress + 1);
};

struct odometry_debug_t {
    position_t raw_left_vel;
    position_t raw_right_vel;
    position_t filt_left_vel;
    position_t filt_right_vel;
    position_t fused_vel;
    double omega_otos = 0.0;
    double omega_imu = 0.0;
    double scalar = 1.0;
    double angular_scalar = 1.0;
    double innovation[5];
    double innovation_sigma[5];
    double cov_diag[6];
    bool left_ok = false;
    bool right_ok = false;
};

void odometrySetup(const odometry_config_t &config);
void odometryUpdate(double dt);
void odometryResetPose(const position_t &pose);
void odometryGetPose(position_t &pos, position_t &vel);

void odometrySetProcessNoise(const ekf_process_noise_t &noise);
void odometrySetMeasurementNoise(const ekf_measurement_noise_t &noise);
void odometrySetGateSigma(double sigma);
void odometrySetLowpassAlpha(float alpha);

void odometryRequestScalar(float scalar);
float odometryGetScalar();
void odometryRequestAngularScalar(float scalar);
float odometryGetAngularScalar();
void odometryStartScalarCalibration();
bool odometryIsScalarCalibrating();

void odometryRequestOffset(const position_t &offset);
position_t odometryGetOffset();

void odometryCalibrateImu();
const odometry_debug_t &odometryDebug();
