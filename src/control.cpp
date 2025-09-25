#include "control.h"
#include "position.h"
#include "Wheel.h"
#include "ctrl_math.h"
#include "clock.h"
#include <math.h>

double errDistance;
double errHeading;
double angleToTarget;

// Updates the speeds of three wheels based on the current and target positions.
// The controller computes the distance and orientation error and then determines
// commanded speeds using proportional gains.
void updateWheels()
{
    // Constant factors
    const double kP_lin = 20.0;   // Gain for linear speed (mm/s per mm error)
    const double kD_lin = 0.9;   // Derivative (mm/s per mm/s error)
    const double kI_lin = 0.0;   // Integral (mm/s per mm*s error)
    const double I_lin_max = 500.0;  // Maximum integral value (mm/s)

    const double kP_ang = 18.0;  // Gain for angular speed (deg/s per deg error)
    const double kD_ang = 0.4;   // Derivative (deg/s per deg/s error)
    const double kI_ang = 5.0;   // Integral (deg/s per deg*s error)
    const double I_ang_max = 100.0;  // Maximum integral value (deg/s)

    const double maxLinSpeed = 1500.0; // mm/s
    const double maxAngSpeed = 500.0;  // deg/s


    // Static variables
    static double linErrorI_x = 0.0;  // Integral value (mm*s)
    static double linErrorI_y = 0.0;  // Integral value (mm*s)
    static double angErrorI = 0.0;  // Integral value (deg*s)
    static uint32_t prevTime = 0;

    // Compute differences in position.
    double dx = global_target.x - global_pos.x; // in mm
    double dy = global_target.y - global_pos.y; // in mm

    errDistance = sqrt(dx * dx + dy * dy);
    errHeading = mod_angle(global_target.a - global_pos.a);
    double lin_speed = sqrt(global_vel.x * global_vel.x + global_vel.y * global_vel.y);  //mm/s
    double ang_speed = global_vel.a;

    // Integrate the error
    double d_t = (double)(get_uptime_us() - prevTime) / 1000000.0;  // in s
    linErrorI_x += d_t * kI_lin * dx;
    linErrorI_y += d_t * kI_lin * dy;
    angErrorI += d_t * kI_ang * errHeading;
    linErrorI_x = fmin(fmax(linErrorI_x, -I_lin_max), I_lin_max);
    linErrorI_y = fmin(fmax(linErrorI_y, -I_lin_max), I_lin_max);
    angErrorI = fmin(fmax(angErrorI, -I_ang_max), I_ang_max);
    prevTime = get_uptime_us();

    double linErrorI = sqrt(linErrorI_x * linErrorI_x + linErrorI_y * linErrorI_y);
    
    // Compute commanded speeds.
    double commandedLin = kP_lin * errDistance + linErrorI - lin_speed * kD_lin; // mm/s
    double commandedAng = kP_ang * errHeading + angErrorI - ang_speed * kD_ang;  // degs/s





    commandedLin = fmin(fmax(commandedLin, -maxLinSpeed), maxLinSpeed); // Limit linear speed
    commandedAng = fmin(fmax(commandedAng, -maxAngSpeed), maxAngSpeed); // Limit angular speed
    
    angleToTarget = mod_angle(global_pos.a - atan2(dy, dx) * 180 / M_PI); // in degrees

    // Update each wheel with the computed linear and angular speed commands.
    wheelA->update(commandedLin, angleToTarget, commandedAng);
    wheelB->update(commandedLin, angleToTarget, commandedAng);
    wheelC->update(commandedLin, angleToTarget, commandedAng);
}