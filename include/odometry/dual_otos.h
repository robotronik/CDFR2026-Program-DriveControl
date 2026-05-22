#pragma once

#include <stdint.h>
#include "types/structs.h"
#include "odometry/OTOS.h"

struct otos_sample_t {
    position_t pos;
    position_t vel;
    position_t acc;
    position_t vel_filtered;
    uint32_t timestamp_us = 0;
    bool ok = false;
};

struct dual_otos_config_t {
    uint8_t address_left = kDefaultAddress;
    uint8_t address_right = (uint8_t)(kDefaultAddress + 1);
    float lowpass_alpha = 0.6f;
};

class DualOtos
{
  public:
    DualOtos();
    bool begin(const dual_otos_config_t &config);
    bool poll(otos_sample_t &left, otos_sample_t &right);
    OTOS *left();
    OTOS *right();
    I2CDevice *leftBus();
    I2CDevice *rightBus();

  private:
    I2CDevice _i2c_left;
    I2CDevice _i2c_right;
    OTOS _otos_left;
    OTOS _otos_right;
    float _alpha;
    position_t _left_vel_filt;
    position_t _right_vel_filt;
};
