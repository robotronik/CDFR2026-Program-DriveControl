#include "position.h"
#include "config.h"
#include "clock.h"
#include "odometry/OTOS.h"
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
    // Exponential filter alpha for velocity
    static const float VELOCITY_ALPHA = 0.8f;
    
    // Apply any pending OTOS parameter changes (from interrupt context)
    // These are applied first, before reading OTOS data
    if (pending_linear_scalar_change) {
        otos->setLinearScalar(pending_linear_scalar);
        pending_linear_scalar_change = false;
        delay_ms(20);
    }
    
    if (pending_angular_scalar_change) {
        otos->setAngularScalar(pending_angular_scalar);
        pending_angular_scalar_change = false;
        delay_ms(20);
    }
    
    if (pending_offset_change) {
        otos->setOffset(pending_offset);
        pending_offset_change = false;
        delay_ms(20);
    }
    
    if (needChangePos) {
        global_pos = newPosition;
        needChangePos = false;
        otos->setPosition(global_pos);
        delay_ms(5); // Give the OTOS time to process the new position
        // Reset velocities for x and y when the position is manually changed
        global_vel.x = 0.0;
        global_vel.y = 0.0;
        _last_time_us = micros();
    }

    global_target = newTarget;
    
    RedLED_Clear();
    // Get the current position from the OTOS
    position_t r_pos, r_vel, r_acc;
    if (otos->getPosVelAcc(r_pos, r_vel, r_acc) == ret_OK) {
        // Use the current global position as the "previous" sample
        double prev_x = global_pos.x;
        double prev_y = global_pos.y;

        // Update the position and acceleration data
        global_pos = r_pos;
        global_acc = r_acc;

        // Compute time delta
        uint32_t now_us = micros();
        double dt = 0.0;
        if (_last_time_us != 0) {
            uint32_t diff = (now_us >= _last_time_us) ? (now_us - _last_time_us) : (0);
            dt = diff / 1e6; // seconds
        }

        if (dt > 0.0) {
            // Instantaneous derivatives using previous global_pos
            double inst_vx = (r_pos.x - prev_x) / dt;
            double inst_vy = (r_pos.y - prev_y) / dt;

            // Exponential filter: new = alpha * old + (1-alpha) * instant
            global_vel.x = VELOCITY_ALPHA * global_vel.x + (1.0f - VELOCITY_ALPHA) * inst_vx;
            global_vel.y = VELOCITY_ALPHA * global_vel.y + (1.0f - VELOCITY_ALPHA) * inst_vy;
        }
        // Leave angular velocity as reported by OTOS
        global_vel.a = r_vel.a;

        // Save time for next derivative
        _last_time_us = now_us;

        static int i = 0;
        if (i == 4){
            usartprintf(">xo:%.1lf,xd:%.1lf\r\n", r_vel.x, global_vel.x);
            i = 0;
        }
        i ++;
    }
    else{
        // If the OTOS is not connected
        RedLED_Set();
    }
    /*
    position_t r_pos_std_dev;
    if (otos->getPositionStdDev(r_pos_std_dev) == ret_OK){
        global_pos_std_dev = r_pos_std_dev;
    }
    else{
        RedLED_Set();
    }
        */
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