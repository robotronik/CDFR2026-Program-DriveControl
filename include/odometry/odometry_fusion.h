#pragma once

#include "types/structs.h"
#include "odometry/dual_otos.h"

struct odometry_config_t {
    float sensor_spacing_mm = 120.0f;
    float lowpass_alpha = 0.75f;
    uint8_t address_left = kDefaultAddress;
    uint8_t address_right = (uint8_t)(kDefaultAddress + 1);
};

struct odometry_debug_t {
    position_t raw_left_vel;
    position_t raw_right_vel;
    position_t filt_left_vel;
    position_t filt_right_vel;
    position_t merged_vel;
    double omega_computed = 0.0;    // Angular velocity from position delta
    double omega_gyro = 0.0;         // Angular velocity from IMU average
    double omega_merged = 0.0;       // Final merged angular velocity
    double scalar = 1.0;
    double angular_scalar = 1.0;
    double scalar_left = 1.0;        // Left sensor calibration scalar
    double scalar_right = 1.0;       // Right sensor calibration scalar
    bool left_ok = false;
    bool right_ok = false;
};

void odometrySetup(const odometry_config_t &config);
void odometryUpdate(double dt);
void odometryResetPose(const position_t &pose);
void odometryGetPose(position_t &pos, position_t &vel);

void odometrySetLowpassAlpha(float alpha);

void odometryRequestScalar(float scalar);
float odometryGetScalar();
void odometryRequestAngularScalar(float scalar);
float odometryGetAngularScalar();

void odometryRequestOffset(const position_t &offset);
position_t odometryGetOffset();

void odometryCalibrateImu();
const odometry_debug_t &odometryDebug();
