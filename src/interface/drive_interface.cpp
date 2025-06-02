#include "drive_interface.h"
#include "led.h"
#include "Motor.h"
#include "position.h"
#include "I2C.h"


i2c_interface::i2c_interface(){

}

uint8_t i2c_interface::get_version() {
    return I2C_VERSION;
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

packed_motion_t i2c_interface::get_motion() {
    packed_motion_t motion;
    motion.pos.x = global_pos.x;
    motion.pos.y = global_pos.y;
    motion.pos.a = global_pos.a;
    motion.vel.x = global_vel.x;
    motion.vel.y = global_vel.y;
    motion.vel.a = global_vel.a;
    motion.acc.x = global_acc.x;
    motion.acc.y = global_acc.y;
    motion.acc.a = global_acc.a;
    return motion;
}

void i2c_interface::set_coordinates(packed_vector_t pos) {
    position_t position;
    position.x = pos.x;
    position.y = pos.y;
    position.a = pos.a;
    setPosition(position);
}

packed_vector_t i2c_interface::get_target() {
    packed_vector_t packed_global_target;
    packed_global_target.x = global_target.x;
    packed_global_target.y = global_target.y;
    packed_global_target.a = global_target.a;
    return (packed_global_target);
}

void i2c_interface::set_target(packed_vector_t pos) {
    position_t position;
    position.x = pos.x;
    position.y = pos.y;
    position.a = pos.a;
    setTarget(position);
}

void i2c_interface::disable() {
    DriveDisable();
}

void i2c_interface::enable() {
    asserv_reset();
    DriveEnable();
}

packed_motor_t i2c_interface::get_current() {
    packed_motor_t current;
    current.motorA = motorA->GetCurrent();
    current.motorB = motorB->GetCurrent();
    current.motorC = motorC->GetCurrent();
    return current;
}

packed_motor_t i2c_interface::get_speed() {
    // TODO
    packed_motor_t speed;
    speed.motorA = 0; // motorA->GetSpeedSigned();
    speed.motorB = 0; // motorB->GetSpeedSigned();
    speed.motorC = 0; // motorC->GetSpeedSigned();
    return speed;
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