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
// The controller now uses independent X/Y PID control with conditional integral accumulation
// for improved stability and faster convergence near the target.
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

    // PID Gains - Linear Control (independent X/Y)
    const double kP_lin = 18.0;   // Proportional (mm/s per mm error)
    const double kI_lin = 30.0;//5.0;    // Integral gain (mm/s per mm*s error)
    const double kD_lin = 0.5;//0.75;   // Derivative (mm/s per mm/s error)

    // PID Gains - Angular Control
    const double kP_ang = 12.0;    // Gain for angular speed (deg/s per deg error)
    const double kI_ang = 0.5;    // Integral gain (deg/s per deg*s error)
    const double kD_ang = 0.5;    // Derivative (deg/s per deg/s error)

    // Limits and thresholds
    const double maxLinSpeed = 2000.0;         // mm/s
    const double maxAngSpeed = 300.0;          // deg/s
    const double maxLinIntegralTerm = maxLinSpeed / 10.0;  // 200 mm/s
    const double maxAngIntegralTerm = maxAngSpeed / 10.0;  // 30 deg/s
    const double fixedDt = 0.004;              // 250 Hz control loop
    
    // Integral accumulation thresholds
    const double integralStartDistance = 50.0;
    const double deadzoneDistance = 1.0;       // Position error considered zero for stopping

    // Compute position errors
    double dx = global_target.x - global_pos.x;  // in mm
    double dy = global_target.y - global_pos.y;  // in mm
    errDistance = sqrt(dx * dx + dy * dy);

    // Compute angular error
    errHeading = mod_angle(global_target.a - global_pos.a);

    // Use raw velocity for derivative term
    double angSpeed = global_vel.a;

    // Dead zone: stop if very close to target
    if (errDistance <= deadzoneDistance) {
        dx = 0.0;
        dy = 0.0;
        errDistance = 0.0;
    }

    // Dead zone for heading
    if (fabs(errHeading) <= 0.4) {
        errHeading = 0.0;
    }

    if (errDistance < integralStartDistance) {
        // Accumulate position error
        linErrorIntegralX += dx * fixedDt;
        linErrorIntegralY += dy * fixedDt;
        
        // Clamp integral state to prevent excessive accumulation
        if (kI_lin > 0.0) {
            double maxIntegralState = maxLinIntegralTerm / kI_lin;
            linErrorIntegralX = fmin(fmax(linErrorIntegralX, -maxIntegralState), maxIntegralState);
            linErrorIntegralY = fmin(fmax(linErrorIntegralY, -maxIntegralState), maxIntegralState);
        }
    } else {
        linErrorIntegralX = 0.0;
        linErrorIntegralY = 0.0;
    }

    // Accumulate angular error (only when not at target heading)
    if (fabs(errHeading) > 0.4) {
        angErrorIntegral += errHeading * fixedDt;
        if (kI_ang > 0.0) {
            double maxAngIntegralState = maxAngIntegralTerm / kI_ang;
            angErrorIntegral = fmin(fmax(angErrorIntegral, -maxAngIntegralState), maxAngIntegralState);
        }
    } else {
        angErrorIntegral = 0.0;  // Reset when at target heading
    }

    // Compute integral term outputs
    double linIntegralTermX = kI_lin * linErrorIntegralX;
    double linIntegralTermY = kI_lin * linErrorIntegralY;

    // Final clamp on integral output
    linIntegralTermX = fmin(fmax(linIntegralTermX, -maxLinIntegralTerm), maxLinIntegralTerm);
    linIntegralTermY = fmin(fmax(linIntegralTermY, -maxLinIntegralTerm), maxLinIntegralTerm);

    double angIntegralTerm = kI_ang * angErrorIntegral;
    angIntegralTerm = fmin(fmax(angIntegralTerm, -maxAngIntegralTerm), maxAngIntegralTerm);

    // ---- INDEPENDENT X/Y CONTROL ----
    // Compute P, I, D terms separately for each axis
    // This preserves the error vector direction and prevents asymmetric behavior
    double commandLinX = kP_lin * dx + linIntegralTermX - global_vel.x * kD_lin;
    double commandLinY = kP_lin * dy + linIntegralTermY - global_vel.y * kD_lin;

    // Apply speed limits to X/Y components
    commandLinX = fmin(fmax(commandLinX, -maxLinSpeed), maxLinSpeed);
    commandLinY = fmin(fmax(commandLinY, -maxLinSpeed), maxLinSpeed);

    // Compute magnitude and direction from X/Y components for wheel commands
    double commandedLin = sqrt(commandLinX * commandLinX + commandLinY * commandLinY);
    double commandedAngle = mod_angle(global_pos.a - atan2(commandLinY, commandLinX) * 180.0 / M_PI);

    // Compute angular command
    double commandAng = kP_ang * errHeading + angIntegralTerm - angSpeed * kD_ang;

    // Apply angular speed limit
    commandAng = fmin(fmax(commandAng, -maxAngSpeed), maxAngSpeed);

    // Update each wheel with the computed linear and angular speed commands.
    wheelA->update(commandedLin, commandedAngle, commandAng);
    wheelB->update(commandedLin, commandedAngle, commandAng);
    wheelC->update(commandedLin, commandedAngle, commandAng);
}