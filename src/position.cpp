#include "position.h"
#include "config.h"
#include "clock.h"
#include "odometry/odometry_fusion.h"
#include "ctrl_math.h" // for mod_angle
#include "led.h"
#include "clock.h"

#include "uart.h"

position_t global_pos = {0.0, 0.0, 0.0}; // mm, mm, deg
position_t global_pos_std_dev = {0.0, 0.0, 0.0}; // mm, mm, deg
position_t global_vel = {0.0, 0.0, 0.0}; // mm/s, mm/s, deg/s
position_t global_acc = {0.0, 0.0, 0.0}; // mm/s^2, mm/s^2, deg/s^2
static position_t newPosition = {0.0, 0.0, 0.0};

position_t global_target = {0.0, 0.0, 0.0};
static position_t newTarget = {0.0, 0.0, 0.0};

static bool needChangePos = false;

// Pending OTOS parameter changes (requested from interrupt context, applied in main loop)
static bool pending_linear_scalar_change = false;
static float pending_linear_scalar = 1.0f;

static bool pending_angular_scalar_change = false;
static float pending_angular_scalar = 1.0f;

static bool pending_offset_change = false;
static position_t pending_offset = {0.0, 0.0, 0.0};

void updatePositionData(){
    // Keep previous position and time to compute derivative
    static uint32_t _last_time_us = 0;
    
    // Apply any pending odometry parameter changes (from interrupt context)
    if (pending_linear_scalar_change) {
        odometryRequestScalar(pending_linear_scalar);
        pending_linear_scalar_change = false;
        usartprintf("Requested linear scalar change: %g\n", pending_linear_scalar);
    }
    
    if (pending_angular_scalar_change) {
        odometryRequestAngularScalar(pending_angular_scalar);
        pending_angular_scalar_change = false;
        usartprintf("Requested angular scalar change: %g\n", pending_angular_scalar);
    }
    
    if (pending_offset_change) {
        odometryRequestOffset(pending_offset);
        pending_offset_change = false;
    }
    
    if (needChangePos) {
        global_pos = newPosition;
        needChangePos = false;
        odometryResetPose(global_pos);
        global_vel.x = 0.0;
        global_vel.y = 0.0;
        global_vel.a = 0.0;
        _last_time_us = micros();
    }

    global_target = newTarget;
    
    RedLED_Clear();
    uint32_t now_us = micros();
    double dt = 0.0;
    if (_last_time_us != 0) {
        uint32_t diff = (now_us >= _last_time_us) ? (now_us - _last_time_us) : (0);
        dt = diff / 1e6;
    }
    if (dt <= 0.0) {
        dt = 0.004;
    }

    odometryUpdate(dt);

    position_t fused_pos;
    position_t fused_vel;
    odometryGetPose(fused_pos, fused_vel);
    global_pos = fused_pos;
    global_vel = fused_vel;
    global_acc = {0.0, 0.0, 0.0};

    const odometry_debug_t &dbg = odometryDebug();
    if (!dbg.left_ok && !dbg.right_ok) {
        RedLED_Set();
    }

    _last_time_us = now_us;
}

void setPosition(position_t incommingPos) {
    newPosition = incommingPos;
    newPosition.a = mod_angle(incommingPos.a);
    needChangePos = true;
}
void setPosition(double x, double y, double a) {
    newPosition.x = x;
    newPosition.y = y;
    newPosition.a = mod_angle(a);
}
void setTarget(position_t incommingPos){
    newTarget = incommingPos;
    newTarget.a = mod_angle(incommingPos.a);
}

void setTarget(double x, double y, double a) {
    newTarget.x = x;
    newTarget.y = y;
    newTarget.a = mod_angle(a);
}

void requestLinearScalarChange(float scalar) {
    pending_linear_scalar = scalar;
    pending_linear_scalar_change = true;
}

void requestAngularScalarChange(float scalar) {
    pending_angular_scalar = scalar;
    pending_angular_scalar_change = true;
}

void requestOffsetChange(position_t offset) {
    pending_offset = offset;
    pending_offset_change = true;
}

float getLinearScalar() {
    return odometryGetScalar();
}

float getAngularScalar() {
    return odometryGetAngularScalar();
}