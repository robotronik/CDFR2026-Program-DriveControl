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
    const double kP_lin = 40.0;   // Gain for linear speed (mm/s per mm error)
    const double kD_lin = 2.2;   // Derivative (mm/s per mm/s error)

    const double kP_ang = 15.0;  // Gain for angular speed (deg/s per deg error)
    const double kD_ang = 0.35;   // Derivative (deg/s per deg/s error)

    const double maxLinSpeed = 1200.0; // mm/s
    const double maxAngSpeed = 500.0;  // deg/s

    // Compute differences in position.
    double dx = global_target.x - global_pos.x; // in mm
    double dy = global_target.y - global_pos.y; // in mm

    errDistance = sqrt(dx * dx + dy * dy);
    errHeading = mod_angle(global_target.a - global_pos.a);
    double lin_speed = sqrt(global_vel.x * global_vel.x + global_vel.y * global_vel.y);  //mm/s
    double ang_speed = global_vel.a;
    
    // Compute commanded speeds.
    double commandedLin = kP_lin * errDistance - lin_speed * kD_lin; // mm/s
    double commandedAng = kP_ang * errHeading - ang_speed * kD_ang;  // degs/s



    commandedLin = fmin(fmax(commandedLin, -maxLinSpeed), maxLinSpeed); // Limit linear speed
    commandedAng = fmin(fmax(commandedAng, -maxAngSpeed), maxAngSpeed); // Limit angular speed
    
    angleToTarget = mod_angle(global_pos.a - atan2(dy, dx) * 180 / M_PI); // in degrees

    // Update each wheel with the computed linear and angular speed commands.
    wheelA->update(commandedLin, angleToTarget, commandedAng);
    wheelB->update(commandedLin, angleToTarget, commandedAng);
    wheelC->update(commandedLin, angleToTarget, commandedAng);
}