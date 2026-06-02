#include "interface/drive_interface.h"
#include "led.h"
#include "Motor.h"
#include "position.h"
#include "I2C.h"
#include "odometry/odometry_fusion.h"
#include "clock.h" // for delay_ms

drive_interface::drive_interface(){}

drive_interface::~drive_interface(){}

uint8_t drive_interface::get_version() {
    return DRIVE_I2C_VERSION;
}

void drive_interface::set_green_led(bool status) {
    if (status)
        GreenLED_Set();
    else
        GreenLED_Clear();
}


void drive_interface::set_red_led(bool status) {
    if (status)
        RedLED_Set();
    else
        RedLED_Clear();
}

packed_motion_t drive_interface::get_motion() {
    packed_motion_t motion;
    motion.pos.x = global_pos.x;
    motion.pos.y = global_pos.y;
    motion.pos.a = global_pos.a;
    motion.vel.x = global_vel.x;
    motion.vel.y = global_vel.y;
    motion.vel.a = global_vel.a;
    return motion;
}

void drive_interface::set_coordinates(packed_position_t pos) {
    position_t position;
    position.x = pos.x;
    position.y = pos.y;
    position.a = pos.a;
    setPosition(position);
}

packed_position_t drive_interface::get_target() {
    packed_position_t packed_global_target;
    packed_global_target.x = global_target.x;
    packed_global_target.y = global_target.y;
    packed_global_target.a = global_target.a;
    return (packed_global_target);
}

void drive_interface::set_target(packed_position_t pos) {
    position_t position;
    position.x = pos.x;
    position.y = pos.y;
    position.a = pos.a;
    setTarget(position);
}

void drive_interface::disable() {
    DriveDisable();
}

void drive_interface::enable() {
    //asserv_reset();
    DriveEnable();
}

void drive_interface::calibrate_otos(){
    odometryCalibrateImu();
}

void drive_interface::calibrate_otos_scalar(){
    // Scalar calibration has been removed in favor of simplified OTOS fusion
    // Set scalars manually using set_linear_scalar() and set_angular_scalar() if needed
}

packed_motor_t drive_interface::get_current() {
    packed_motor_t current;
    current.A = motorA->GetCurrent();
    current.B = motorB->GetCurrent();
    current.C = motorC->GetCurrent();
    return current;
}

packed_motor_t drive_interface::get_speed() {
    // TODO
    packed_motor_t speed;
    speed.A = 0; // motorA->GetSpeedSigned();
    speed.B = 0; // motorB->GetSpeedSigned();
    speed.C = 0; // motorC->GetSpeedSigned();
    return speed;
}

void drive_interface::set_brake_state(bool brakeEnable) {
    if (brakeEnable) {
        motorA->Brake(true);
        motorB->Brake(true);
        motorC->Brake(true);
    }
    else {
        //robotAsserv->reset();
        motorA->SetSpeedSigned(0);
        motorB->SetSpeedSigned(0);
        motorC->SetSpeedSigned(0);
        motorA->Brake(false);
        motorB->Brake(false);
        motorC->Brake(false);
    }
}

void drive_interface::set_max_torque(double current){
    // Reported to 10.8 amps, TODO change
    int val = (int)(current / 10.8 * 100.0);
    motorA->SetMaxTorque(val);
    motorB->SetMaxTorque(val);
    motorC->SetMaxTorque(val);
}

void drive_interface::set_linear_scalar(float scalar) {
    // Request scalar change - will be applied synchronously in updatePositionData
    requestLinearScalarChange(scalar);
}

float drive_interface::get_linear_scalar() {
    return getLinearScalar();
}

void drive_interface::set_angular_scalar(float scalar) {
    // Request scalar change - will be applied synchronously in updatePositionData
    requestAngularScalarChange(scalar);
}

float drive_interface::get_angular_scalar() {
    return 1.0f; // TODO Remove
}

void drive_interface::set_offset(position_t offset) {
    // Offset support removed - this is now a no-op
    (void)offset; // Suppress unused parameter warning
}

position_t drive_interface::get_offset() {
    // Offset support removed - return zero offset
    return {0.0, 0.0, 0.0};
}

status_t drive_interface::get_status() {
    status_t status;
    status.is_error1 = false;
    status.is_error2 = false;
    return status;
}