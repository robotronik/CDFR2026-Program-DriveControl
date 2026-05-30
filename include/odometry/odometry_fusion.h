#pragma once

#include "types/structs.h"

struct odometry_config_t {
    float sensor_spacing_mm;
    float lowpass_alpha;
    uint8_t address_left;
    uint8_t address_right;
};

void odometrySetup(const odometry_config_t &config);
void odometryUpdate(double dt);
void odometryResetPose(position_t pose);

void odometrySetLowpassAlpha(float alpha);

void odometryRequestScalar(float scalar);
float odometryGetScalar();
void odometryRequestAngularScalar(float scalar);
float odometryGetAngularScalar();

void odometryCalibrateImu();
void odometryDebugPrint();
