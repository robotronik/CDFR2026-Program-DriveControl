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
static position_t g_pos_l, g_vel_l, g_acc_l;
static position_t g_pos_r, g_vel_r, g_acc_r;

// Store config for calibration functions
static float g_sensor_spacing_mm = 0.0f;

bool is_odometry_ok = false;

void odometrySetup(const odometry_config_t &config)
{
    // Store sensor spacing for calibration
    g_sensor_spacing_mm = config.sensor_spacing_mm;
    
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
    // Start calibration on both sensors without waiting
    otos_left.calibrateImu(255, false);
    otos_right.calibrateImu(255, false);
    
    // Wait for both sensors to finish calibration
    uint8_t samples_left = 1;
    uint8_t samples_right = 1;
    
    while (samples_left > 0 || samples_right > 0) {
        otos_left.getImuCalibrationProgress(samples_left);
        otos_right.getImuCalibrationProgress(samples_right);
        
        if (samples_left > 0 || samples_right > 0) {
            delay_ms(20);  // Small delay to avoid hammering the I2C bus
        }
    }
}

double averageAngle(double a1, double a2) {
    double x = cos(a1 * kDegToRad) + cos(a2 * kDegToRad);
    double y = sin(a1 * kDegToRad) + sin(a2 * kDegToRad);
    return atan2(y, x) * kRadToDeg;
}

float calculateLinearScalarCorrection() {
    // Translate sensor positions relative to reported robot center
    double sensor_l_rel_x = g_pos_l.x - global_pos.x;
    double sensor_l_rel_y = g_pos_l.y - global_pos.y;
    double sensor_r_rel_x = g_pos_r.x - global_pos.x;
    double sensor_r_rel_y = g_pos_r.y - global_pos.y;
    
    // Rotate back to each sensor's own frame using its reported angle
    double cos_al = cos(-g_pos_l.a * kDegToRad);
    double sin_al = sin(-g_pos_l.a * kDegToRad);
    double cos_ar = cos(-g_pos_r.a * kDegToRad);
    double sin_ar = sin(-g_pos_r.a * kDegToRad);
    
    // In left sensor's frame, its offset is (0, +spacing/2)
    double sensor_l_robot_y = sensor_l_rel_x * sin_al + sensor_l_rel_y * cos_al + (g_sensor_spacing_mm / 2.0);
    
    // In right sensor's frame, its offset is (0, -spacing/2)
    double sensor_r_robot_y = sensor_r_rel_x * sin_ar + sensor_r_rel_y * cos_ar - (g_sensor_spacing_mm / 2.0);
    
    // Both should now be ~0 if scalars are correct (they measure from their own center)
    // The reported y separation should be: spacing/2 - (-spacing/2) = spacing
    double reported_y_separation = sensor_l_robot_y - sensor_r_robot_y;

    usartprintf("Sensor L robot y: %.2f, Sensor R robot y: %.2f, y separation: %.2f\r\n",
        sensor_l_robot_y, sensor_r_robot_y, reported_y_separation);
    
    // The correction factor is: expected_spacing / reported_y_separation
    // If reported < expected, scalars were too low -> increase
    // If reported > expected, scalars were too high -> decrease
    if (reported_y_separation > 0.001) {  // Avoid division by near-zero
        float correction = g_sensor_spacing_mm / reported_y_separation;
        return correction;
    }
    return 1.0f;  // No correction if separation is too small
}

void odometryUpdate(double dt)
{

    // Read position, velocity, and acceleration from both sensors
    bool left_ok = otos_left.getPosVelAcc(g_pos_l, g_vel_l, g_acc_l) == ret_OK;
    bool right_ok = otos_right.getPosVelAcc(g_pos_r, g_vel_r, g_acc_r) == ret_OK;

    if (!left_ok || !right_ok) {
        // Not enough sensors, cannot update
        is_odometry_ok = false;
        return;
    }
    
    // Sensors are responding
    is_odometry_ok = true;

    position_t prev_global_pos = global_pos;
    
    // Simply merge both sensors by averaging their absolute positions
    global_pos.x = 0.5 * (g_pos_l.x + g_pos_r.x);
    global_pos.y = 0.5 * (g_pos_l.y + g_pos_r.y);
    
    // Average angle from both sensors for heading
    global_pos.a = averageAngle(g_pos_l.a, g_pos_r.a);
    
    // Merge velocity from both sensors
    float alpha = 0.25f;  // Smoothing factor for velocity
    global_vel.x = alpha * (global_pos.x - prev_global_pos.x) / dt + (1.0f - alpha) * global_vel.x;
    global_vel.y = alpha * (global_pos.y - prev_global_pos.y) / dt + (1.0f - alpha) * global_vel.y;   
    global_vel.a = 0.5f * (g_vel_l.a + g_vel_r.a);  // Average angular velocity

    static int i = 0;
    if (i > 1000 && fabs(global_vel.x) < 0.01f && fabs(global_vel.y) < 0.01f && fabs(global_vel.a) < 0.1f) {
        i = 0;
        // Set both sensors to the same position to correct for drift, using the current global position as reference
        odometryResetPose(global_pos);
    }
    i++;
}

void odometryResetPose(position_t pose)
{
    // Write to the otos the new pose
    otos_left.setPosition(pose);
    otos_right.setPosition(pose);
    global_pos = pose;  // Update global position immediately to avoid jumps until next sensor read
}

void odometryDebugPrint()
{
    //float corr = calculateLinearScalarCorrection();
    usartprintf(">x:%.2lf,y:%.2lf,a:%.2lf,vx:%.2lf,vy:%.2lf,va:%.2lf,lx:%.2lf,ly:%.2lf,la:%.2lf,rx:%.2lf,ry:%.2lf,ra:%.2lf\r\n",
        global_pos.x, global_pos.y, global_pos.a,
        global_vel.x, global_vel.y, global_vel.a,
        g_pos_l.x, g_pos_l.y, g_pos_l.a,
        g_pos_r.x, g_pos_r.y, g_pos_r.a);
    // Print postion and velocity from both sensors for debugging, in a single line for easier parsing
    /*usartprintf(">Lx:%.2lf,Ly:%.2lf,La:%.2lf,Rx:%.2lf,Ry:%.2lf,Ra:%.2lf,Lvx:%.2lf,Lvy:%.2lf,Lva:%.2lf,Rvx:%.2lf,Rvy:%.2lf,Rva:%.2lf\r\n",
        g_pos_l.x, g_pos_l.y, g_pos_l.a,
        g_pos_r.x, g_pos_r.y, g_pos_r.a,
        g_vel_l.x, g_vel_l.y, g_vel_l.a,
        g_vel_r.x, g_vel_r.y, g_vel_r.a);*/
}