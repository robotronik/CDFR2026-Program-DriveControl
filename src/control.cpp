#include "control.h"
#include "position.h"
#include "Wheel.h"
#include "ctrl_math.h"
#include <math.h>

double errDistance;
double errHeading;
double angleToTarget;

// Updates the speeds of three wheels based on the current and target positions.
// The controller computes the distance and orientation error and then determines
// commanded speeds using proportional gains.
void updateWheels()
{
    // Compute differences in position.
    double dx = global_target.x - global_pos.x; // in mm
    double dy = global_target.y - global_pos.y; // in mm

    errDistance = sqrt(dx * dx + dy * dy);
    errHeading = mod_angle(global_target.a - global_pos.a);

    const double kP_linear = 10.0;   // Gain for linear speed (mm/s per mm error)
    const double kP_angular = 10.0;  // Gain for angular speed (deg/s per deg error)
    
    // Compute commanded speeds.
    double commandedLinear = kP_linear * errDistance; // mm/s
    double commandedAngular = kP_angular * errHeading;  // degs/s

    const double maxLinearSpeed = 1000.0; // mm/s
    const double maxAngularSpeed = 360.0;  // deg/s

    commandedLinear = fmin(fmax(commandedLinear, -maxLinearSpeed), maxLinearSpeed); // Limit linear speed
    commandedAngular = fmin(fmax(commandedAngular, -maxAngularSpeed), maxAngularSpeed); // Limit angular speed
    
    angleToTarget = mod_angle(global_pos.a - atan2(dy, dx) * 180 / M_PI); // in degrees

    // Update each wheel with the computed linear and angular speed commands.
    wheelA->update(commandedLinear, angleToTarget, commandedAngular);
    wheelB->update(commandedLinear, angleToTarget, commandedAngular);
    wheelC->update(commandedLinear, angleToTarget, commandedAngular);

    /*
    Serial.print("commandedLinear");
    Serial.print(commandedLinear);
    Serial.print("commandedAngular");
    Serial.print(commandedAngular);
    */
}