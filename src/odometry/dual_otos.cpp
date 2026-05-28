#include "odometry/dual_otos.h"
#include "clock.h"

DualOtos::DualOtos()
    : _i2c_left(kDefaultAddress),
      _i2c_right((uint8_t)(kDefaultAddress + 1)),
      _otos_left(&_i2c_left),
      _otos_right(&_i2c_right)
{
    _alpha = 0.6f;
    _left_vel_filt = {0.0, 0.0, 0.0};
    _right_vel_filt = {0.0, 0.0, 0.0};
}

bool DualOtos::begin(const dual_otos_config_t &config)
{
    _i2c_left.setAddress(config.address_left);
    _i2c_right.setAddress(config.address_right);
    _alpha = config.lowpass_alpha;

    bool left_ok = (_otos_left.begin() == ret_OK);
    bool right_ok = (_otos_right.begin() == ret_OK);

    return left_ok && right_ok;
}

#include "uart.h"
bool DualOtos::poll(otos_sample_t &left, otos_sample_t &right)
{
    position_t pos_l, vel_l, acc_l;
    position_t pos_r, vel_r, acc_r;

    left.ok = (_otos_left.getPosVelAcc(pos_l, vel_l, acc_l) == ret_OK);
    right.ok = (_otos_right.getPosVelAcc(pos_r, vel_r, acc_r) == ret_OK);

    left.timestamp_us = micros();
    right.timestamp_us = left.timestamp_us;

    if (left.ok) {
        left.pos = pos_l;
        left.vel = vel_l;
        left.acc = acc_l;

        _left_vel_filt.x = _alpha * _left_vel_filt.x + (1.0f - _alpha) * vel_l.x;
        _left_vel_filt.y = _alpha * _left_vel_filt.y + (1.0f - _alpha) * vel_l.y;
        _left_vel_filt.a = _alpha * _left_vel_filt.a + (1.0f - _alpha) * vel_l.a;
        left.vel_filtered = _left_vel_filt;
    }

    if (right.ok) {
        right.pos = pos_r;
        right.vel = vel_r;
        right.acc = acc_r;

        _right_vel_filt.x = _alpha * _right_vel_filt.x + (1.0f - _alpha) * vel_r.x;
        _right_vel_filt.y = _alpha * _right_vel_filt.y + (1.0f - _alpha) * vel_r.y;
        _right_vel_filt.a = _alpha * _right_vel_filt.a + (1.0f - _alpha) * vel_r.a;
        right.vel_filtered = _right_vel_filt;
    }

    return left.ok || right.ok;
}

OTOS *DualOtos::left()
{
    return &_otos_left;
}

OTOS *DualOtos::right()
{
    return &_otos_right;
}

I2CDevice *DualOtos::leftBus()
{
    return &_i2c_left;
}

I2CDevice *DualOtos::rightBus()
{
    return &_i2c_right;
}
