#include "control.h"
#include "position.h"
#include "Wheel.h"
#include "ctrl_math.h"
#include <math.h>

double errDistance;
double errHeading;
double angleToTarget;

static double linErrorIntegralX = 0.0;
static double linErrorIntegralY = 0.0;
static double angErrorIntegral = 0.0;

// Updates the speeds of three wheels based on the current and target positions.
// The controller computes the distance and orientation error and then determines
// commanded speeds using proportional gains.
void updateWheels()
{
    if (isDriveEnabled() == false) {
        linErrorIntegralX = 0.0;
        linErrorIntegralY = 0.0;
        angErrorIntegral = 0.0;
        wheelA->setSpeed(0.0);
        wheelB->setSpeed(0.0);
        wheelC->setSpeed(0.0);
        return;
    }
    // Constant factors
    const double kP_lin = 15.0;   // Gain for linear speed (mm/s per mm error)
    const double kI_lin = 5.0;    // Integral gain (mm/s per mm*s error)
    const double kD_lin = 2.0;   // Derivative (mm/s per mm/s error)

    const double kP_ang = 4.0;  // Gain for angular speed (deg/s per deg error)
    const double kI_ang = 0.5;   // Integral gain (deg/s per deg*s error)
    const double kD_ang = 0.5;   // Derivative (deg/s per deg/s error)

    const double maxLinSpeed = 2000.0; // mm/s
    const double maxAngSpeed = 300.0;  // deg/s
    const double maxLinIntegralTerm = maxLinSpeed / 10.0;
    const double maxAngIntegralTerm = maxAngSpeed / 10.0;
    const double fixedDt = 0.004;       // 250 Hz control loop

    // Compute differences in position.
    double dx = global_target.x - global_pos.x; // in mm
    double dy = global_target.y - global_pos.y; // in mm

    errDistance = sqrt(dx * dx + dy * dy);
    errHeading = mod_angle(global_target.a - global_pos.a);
    double lin_speed = sqrt(global_vel.x * global_vel.x + global_vel.y * global_vel.y);  //mm/s
    double ang_speed = global_vel.a;

    if (errDistance <= 1.0) {
        // If we are very close to the target, stop the robot to avoid oscillations.
        errDistance = 0.0;
        linErrorIntegralX = 0.0;
        linErrorIntegralY = 0.0;
    }

    if (fabs(errHeading) <= 0.4) {
        errHeading = 0.0;
        angErrorIntegral = 0.0;
    }

    linErrorIntegralX += dx * fixedDt;
    linErrorIntegralY += dy * fixedDt;

    if (kI_lin > 0.0) {
        double maxIntegralState = maxLinIntegralTerm / kI_lin;
        linErrorIntegralX = fmin(fmax(linErrorIntegralX, -maxIntegralState), maxIntegralState);
        linErrorIntegralY = fmin(fmax(linErrorIntegralY, -maxIntegralState), maxIntegralState);
    }

    angErrorIntegral += errHeading * fixedDt;
    if (kI_ang > 0.0) {
        double maxAngIntegralState = maxAngIntegralTerm / kI_ang;
        angErrorIntegral = fmin(fmax(angErrorIntegral, -maxAngIntegralState), maxAngIntegralState);
    }

    double linIntegralTermX = kI_lin * linErrorIntegralX;
    double linIntegralTermY = kI_lin * linErrorIntegralY;

    linIntegralTermX = fmin(fmax(linIntegralTermX, -maxLinIntegralTerm), maxLinIntegralTerm);
    linIntegralTermY = fmin(fmax(linIntegralTermY, -maxLinIntegralTerm), maxLinIntegralTerm);

    double angIntegralTerm = kI_ang * angErrorIntegral;
    angIntegralTerm = fmin(fmax(angIntegralTerm, -maxAngIntegralTerm), maxAngIntegralTerm);
    
    double commandedLinX = kP_lin * dx + linIntegralTermX - global_vel.x * kD_lin; // mm/s
    double commandedLinY = kP_lin * dy + linIntegralTermY - global_vel.y * kD_lin; // mm/s

    double commandedLin = sqrt(commandedLinX * commandedLinX + commandedLinY * commandedLinY);
    double commandedAng = kP_ang * errHeading + angIntegralTerm - ang_speed * kD_ang;  // degs/s

    commandedLin = fmin(fmax(commandedLin, -maxLinSpeed), maxLinSpeed); // Limit linear speed
    commandedAng = fmin(fmax(commandedAng, -maxAngSpeed), maxAngSpeed); // Limit angular speed
    
    angleToTarget = mod_angle(global_pos.a - atan2(dy, dx) * 180 / M_PI); // in degrees

    // Update each wheel with the computed linear and angular speed commands.
    wheelA->update(commandedLin, angleToTarget, commandedAng);
    wheelB->update(commandedLin, angleToTarget, commandedAng);
    wheelC->update(commandedLin, angleToTarget, commandedAng);
}