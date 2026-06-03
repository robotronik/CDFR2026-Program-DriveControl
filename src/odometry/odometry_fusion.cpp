#include "odometry/odometry_fusion.h"
#include "odometry/OTOS.h"
#include "odometry/I2Cdevice.h"
#include "ctrl_math.h"
#include "clock.h"
#include "uart.h"
#include <math.h>

static constexpr double kDegToRad = PI / 180.0;
static constexpr double kRadToDeg = 180.0 / PI;

// OTOS sensor instances
static I2CDevice g_i2c_left(kDefaultAddress);
static I2CDevice g_i2c_right((uint8_t)(kDefaultAddress + 1));
static OTOS otos_left(&g_i2c_left);
static OTOS otos_right(&g_i2c_right);

// External global position and velocity (set by odometryUpdate)
extern position_t global_pos;
extern position_t global_vel;

// Debug: individual sensor positions
static position_t g_pos_l;
static position_t g_pos_r;

bool is_odometry_ok = false;

void odometrySetup(const odometry_config_t &config)
{
    // Initialize I2C devices with configured addresses
    g_i2c_left.setAddress(config.address_left);
    g_i2c_right.setAddress(config.address_right);

    // Initialize OTOS sensors
    otos_left.begin();
    otos_right.begin();

    // Wait for OTOS sensors to be connected
    usartprintf("Looking for OTOS left/right...\n");
    while ((otos_left.isConnected() != ret_OK) || (otos_right.isConnected() != ret_OK)) {
        usartprintf("OTOS not connected\n");
        delay_ms(200);
    }

    // Wait for OTOS sensors to pass self-test
    while ((otos_left.selfTest() != ret_OK) || (otos_right.selfTest() != ret_OK)) {
        usartprintf("OTOS self-test failed\n");
        delay_ms(200);
    }
    usartprintf("OTOS connected\n");
    delay_ms(100);

    // Set OTOS offset and calibration scalars
    // Each sensor gets its respective offset
    position_t offset_left = {0.0f, (float)config.sensor_spacing_mm / 2.0f, 0.0f};
    position_t offset_right = {0.0f, -(float)config.sensor_spacing_mm / 2.0f, 0.0f};
    otos_left.setOffset(offset_left);
    otos_right.setOffset(offset_right);
    // Reduce linear scalar when reporting more than it did
    otos_left.setLinearScalar(0.940f);
    otos_right.setLinearScalar(0.960f);
    otos_left.setAngularScalar(0.992f);
    otos_right.setAngularScalar(0.992f);

    odometryCalibrateImu();
    usartprintf("OTOS IMU calibration requested\n");
    delay_ms(100);

    // Mark odometry as operational
    is_odometry_ok = true;

    otos_signal_process_config_t signal_config;
    signal_config.enLut = 1;
    signal_config.enRot = 1;
    signal_config.enAcc = 1;
    signal_config.enVar = 1;
    otos_left.setSignalProcessConfig(signal_config);
    otos_right.setSignalProcessConfig(signal_config);

    otos_left.resetTracking();
    otos_right.resetTracking();
}

void odometryRequestScalar(float scalar)
{
    return;
}

float odometryGetScalar()
{
    return 1.0f;
}

void odometryRequestAngularScalar(float scalar)
{
    otos_left.setAngularScalar(scalar);
    otos_right.setAngularScalar(scalar);
}



void odometryCalibrateImu()
{
    otos_left.calibrateImu(255, false);
    otos_right.calibrateImu(255, false);
}

double averageAngle(double a1, double a2) {
    double x = cos(a1 * kDegToRad) + cos(a2 * kDegToRad);
    double y = sin(a1 * kDegToRad) + sin(a2 * kDegToRad);
    return atan2(y, x) * kRadToDeg;
}

void odometryUpdate(double dt)
{
    position_t vel_l, acc_l;
    position_t vel_r, acc_r;

    // Read position, velocity, and acceleration from both sensors
    bool left_ok = otos_left.getPosVelAcc(g_pos_l, vel_l, acc_l) == ret_OK;
    bool right_ok = otos_right.getPosVelAcc(g_pos_r, vel_r, acc_r) == ret_OK;

    if (!left_ok || !right_ok) {
        // Not enough sensors, cannot update
        is_odometry_ok = false;
        return;
    }
    
    // Sensors are responding
    is_odometry_ok = true;
    
    // Simply merge both sensors by averaging their absolute positions
    global_pos.x = 0.5 * (g_pos_l.x + g_pos_r.x);
    global_pos.y = 0.5 * (g_pos_l.y + g_pos_r.y);
    
    // Average angle from both sensors for heading
    global_pos.a = averageAngle(g_pos_l.a, g_pos_r.a);
    
    // Merge velocity from both sensors
    global_vel.x = 0.5 * (vel_l.x + vel_r.x);
    global_vel.y = 0.5 * (vel_l.y + vel_r.y);
    global_vel.a = 0.5 * (vel_l.a + vel_r.a);  // Average angular velocity
}

void odometryResetPose(position_t pose)
{
    // Write to the otos the new pose
    otos_left.setPosition(pose);
    otos_right.setPosition(pose);
}

void odometryDebugPrint()
{
    usartprintf(">x:%.2lf,y:%.2lf,a:%.2lf,vx:%.2lf,vy:%.2lf,va:%.2lf,lx:%.2lf,ly:%.2lf,la:%.2lf,rx:%.2lf,ry:%.2lf,ra:%.2lf\r\n",
        global_pos.x, global_pos.y, global_pos.a,
        global_vel.x, global_vel.y, global_vel.a,
        g_pos_l.x, g_pos_l.y, g_pos_l.a,
        g_pos_r.x, g_pos_r.y, g_pos_r.a);
}