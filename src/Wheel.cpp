#include <math.h>
#include "Wheel.h"
#include "config.h"
#include "types/structs.h"

Wheel* wheelA = nullptr; // WheelA at 0°
Wheel* wheelB = nullptr; // WheelB at 120°
Wheel* wheelC = nullptr; // WheelC at 240°

Wheel::Wheel(double dist, double ang, double diameter, Motor* motorPtr) :
    distanceToCenter(dist), angle(ang), diam(diameter), motor(motorPtr) 
{
    // Constructor implementation
}


// Updates the speed of a single wheel given the commandd linear and angular velocities.
// linear: linear speed of robot in mm/s
// theta: current error angle to target in degrees
// angular: angular speed of robot in degrees/s
void Wheel::update(double linear, double theta, double angular) {
    double rad = (angle + theta) * DEG_TO_RAD;

    double circumference = PI * diam; // in mm
    double robotCircumference = 2 * PI * distanceToCenter; // in mm
    
    // Compute the wheel speed by projecting the commandd linear velocity and adding the contribution of rotational velocity.
    // The wheel speed is in mm/s.
    double wheelSpeed = linear * -sin(rad) + robotCircumference * angular / 360;

    // Convert the wheel speed to revolutions per second (rps).
    double wheelSpeedRPS = wheelSpeed / circumference; // in rps
    // Set the speed of the motor.
    setSpeed(wheelSpeedRPS); // Set the speed of the motor
}

// speed is in rps
void Wheel::setSpeed(double speed) {
    double maxSpeed = 6.8; // in rps  (400 rpm motors)
    int speedPercent = 100 * speed / maxSpeed;
    motor->SetSpeedSigned(speedPercent); // Set the speed of the motor
}