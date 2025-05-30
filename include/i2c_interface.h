#pragma once

#include "robot_interface.h"
#include "movement.h"
#include "I2C.h"

#define STRUCT_PACK __attribute__((__packed__))

typedef struct {
    bool is_error1:1;
    bool is_error2:1;
} STRUCT_PACK status_t;

class i2c_interface : public Robot_interface
{
private:
    movement* robotAsserv;
public:
    i2c_interface(movement* inRobotAsservisement);
    ~i2c_interface(){};

    void get_version(uint16_t version);
    void set_green_led(bool status);
    void set_red_led(bool status);
    void get_coordinates(position_t &pos, position_t &vel, position_t &acc);
    void set_coordinates(position_t pos);
    void set_target(position_t pos);
    void get_target(position_t &pos);

    void disable();
    void enable();

    void get_current(int16_t &currentRight, int16_t &currentLeft);
    void get_speed(int16_t &speedRight, int16_t &speedLeft);

    void set_brake_state(bool enable);

    void set_max_torque(double current);
    
    void get_status(status_t& status);

    void setReponseBuffer(uint8_t* data, int size);
};


