#include "position.h"
#include "config.h"
#include "clock.h"
#include "odometry/OTOS.h"
#include "ctrl_math.h" // for mod_angle
#include "led.h"
#include "clock.h"

position_t global_pos = {0.0, 0.0, 0.0}; // mm, mm, deg
position_t global_pos_std_dev = {0.0, 0.0, 0.0}; // mm, mm, deg
position_t global_vel = {0.0, 0.0, 0.0}; // mm/s, mm/s, deg/s
position_t global_acc = {0.0, 0.0, 0.0}; // mm/s^2, mm/s^2, deg/s^2
static position_t newPosition = {0.0, 0.0, 0.0};

position_t global_target = {0.0, 0.0, 0.0};
static position_t newTarget = {0.0, 0.0, 0.0};

static bool needChangePos = false;

void updatePositionData(){
    if (needChangePos) {
        global_pos = newPosition;
        needChangePos = false;
        otos->setPosition(global_pos);
        delay_ms(5); // Give the OTOS time to process the new position
    }

    global_target = newTarget;
    
    RedLED_Clear();
    // Get the current position from the OTOS
    position_t r_pos, r_vel, r_acc;
    if (otos->getPosVelAcc(r_pos, r_vel, r_acc) == ret_OK) {
        // Update the position, velocity, and acceleration data
        global_pos = r_pos;
        global_vel = r_vel;
        global_acc = r_acc;
    }
    else{
        // If the OTOS is not connected
        RedLED_Set();
    }
    position_t r_pos_std_dev;
    if (otos->getPositionStdDev(r_pos_std_dev) == ret_OK){
        global_pos_std_dev = r_pos_std_dev;
    }
    else{
        RedLED_Set();
    }
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
    global_target = newTarget;
}

void setTarget(double x, double y, double a) {
    newTarget.x = x;
    newTarget.y = y;
    newTarget.a = mod_angle(a);
    global_target = newTarget;
}