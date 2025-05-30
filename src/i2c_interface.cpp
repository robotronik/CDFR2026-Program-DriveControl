#include "i2c_interface.h"
#include "led.h"
#include "Motor.h"
#include "position.h"

i2c_interface::i2c_interface(){

}

void i2c_interface::get_version(uint16_t &part1) {
    //////
}

void i2c_interface::setReponseBuffer(uint8_t* data, int size) {
    I2CSetBuffer(data,size);
}

void i2c_interface::set_green_led(bool status) {
    if (status)
        GreenLED_Set();
    else
        GreenLED_Clear();
}


void i2c_interface::set_red_led(bool status) {
    if (status)
        RedLED_Set();
    else
        RedLED_Clear();
}

void i2c_interface::get_coordinates(position_t &pos, position_t &vel, position_t &acc) {
    pos = global_pos;
    vel = global_vel;
    acc = global_acc;
}

void i2c_interface::set_coordinates(position_t pos) {
    setPosition(pos);
}

void i2c_interface::get_target(position_t& pos) {
    pos = global_target;
}

void i2c_interface::set_target(position_t pos) {
    setTarget(pos);
}

void i2c_interface::disable() {
    DriveDisable();
}

void i2c_interface::enable() {
    asserv_reset();
    DriveEnable();
}

void i2c_interface::get_current(int16_t &currentRight, int16_t &currentLeft) {
    currentLeft = motorA->GetCurrent();
}

void i2c_interface::get_speed(int16_t &speedRight, int16_t &speedLeft) {
    // TODO
}

void i2c_interface::set_brake_state(bool brakeEnable) {
    if (brakeEnable) {
        motorA->Brake(true);
        motorB->Brake(true);
        motorC->Brake(true);
    }
    else {
        robotAsserv->reset();
        motorA->SetSpeedSigned(0);
        motorB->SetSpeedSigned(0);
        motorC->SetSpeedSigned(0);
        motorA->Brake(false);
        motorB->Brake(false);
        motorC->Brake(false);
    }
}

void i2c_interface::set_max_torque(double current){
    // Reported to 10.8 amps, TODO change
    int val = (int)(current / 10.8 * 100.0);
    motorA->SetMaxTorque(val);
    motorB->SetMaxTorque(val);
    motorC->SetMaxTorque(val);
}

void i2c_interface::get_status(status_t& status) {
    status.is_error1 = false;
    status.is_error2 = false;
}