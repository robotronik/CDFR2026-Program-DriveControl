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
static OTOS g_otos_left(&g_i2c_left);
static OTOS g_otos_right(&g_i2c_right);
static odometry_config_t g_cfg;

// Debug tracking variables
static position_t g_debug_imu_left_vel = {0.0, 0.0, 0.0};
static position_t g_debug_imu_right_vel = {0.0, 0.0, 0.0};
static double angle_from_dx = 0.0;

// External global position and velocity (set by odometryUpdate)
extern position_t global_pos;
extern position_t global_vel;

// Internal state: filtered velocity
static position_t filtered_vel_left = {0.0, 0.0, 0.0};
static position_t filtered_vel_right = {0.0, 0.0, 0.0};

// Previous sensor positions for delta calculation
static position_t g_prev_pos_left = {0.0, 0.0, 0.0};
static position_t g_prev_pos_right = {0.0, 0.0, 0.0};

bool is_odometry_ok = false;

static float g_scalar_left = 1.0f;
static float g_scalar_right = 1.0f;
static float g_angular_scalar = 1.0f;

// Auto-calibration during rotation
static double g_calib_left_sum = 0.0;
static double g_calib_right_sum = 0.0;
static int g_calib_left_count = 0;
static int g_calib_right_count = 0;
static constexpr double CALIB_MAX_LINEAR_VEL = 20.0;   // mm/s
static constexpr double CALIB_MIN_ANGULAR_VEL = 50.0;  // deg/s

void odometrySetup(const odometry_config_t &config)
{
    g_cfg = config;

    // Initialize I2C devices with configured addresses
    g_i2c_left.setAddress(config.address_left);
    g_i2c_right.setAddress(config.address_right);

    // Initialize OTOS sensors
    g_otos_left.begin();
    g_otos_right.begin();

    // Update global pointers for external access
    otosLeft = &g_otos_left;
    otosRight = &g_otos_right;
    i2cDeviceLeft = &g_i2c_left;
    i2cDeviceRight = &g_i2c_right;

    // Wait for OTOS sensors to be connected
    usartprintf("Looking for OTOS left/right...\n");
    while ((otosLeft == nullptr) || (otosRight == nullptr) ||
           (otosLeft->isConnected() != ret_OK) || (otosRight->isConnected() != ret_OK)) {
        usartprintf("OTOS not connected\n");
        delay_ms(200);
    }

    // Wait for OTOS sensors to pass self-test
    while ((otosLeft->selfTest() != ret_OK) || (otosRight->selfTest() != ret_OK)) {
        usartprintf("OTOS self-test failed\n");
        delay_ms(200);
    }
    usartprintf("OTOS connected\n");
    delay_ms(100);

    // Set OTOS offset and calibration scalars (hardcoded defaults)
    position_t offset = {0.0, 0.0, 0.0};
    float linear_scalar = 0.999f;
    float angular_scalar = 0.999f;
    
    otosLeft->setOffset(offset);
    otosRight->setOffset(offset);
    otosLeft->setLinearScalar(linear_scalar);
    otosRight->setLinearScalar(linear_scalar);
    otosLeft->setAngularScalar(angular_scalar);
    otosRight->setAngularScalar(angular_scalar);

    // Reset tracking and calibrate IMU
    otosLeft->resetTracking();
    otosRight->resetTracking();
    odometryCalibrateImu();
    usartprintf("OTOS IMU calibration requested\n");
    delay_ms(100);

    // Mark odometry as operational
    is_odometry_ok = true;

    // Configure signal processing: disable rotation and Kalman filter
    if (otosLeft) {
        otos_signal_process_config_t signal_config;
        if (otosLeft->getSignalProcessConfig(signal_config) == ret_OK) {
            signal_config.enLut = 1;   // Disable internal lookup table calibration for optical sensor
            signal_config.enRot = 0;   // Disable rotation - keep IMU and optical data in sensor frame
            signal_config.enAcc = 1;   // Disable accelerometer data in Kalman filter
            signal_config.enVar = 1;   // Disable Kalman filter (set variance to 0) to get raw integrated position
            otosLeft->setSignalProcessConfig(signal_config);
        }
    }
    if (otosRight) {
        otos_signal_process_config_t signal_config;
        if (otosRight->getSignalProcessConfig(signal_config) == ret_OK) {
            signal_config.enLut = 1;   // Disable internal lookup table calibration for optical sensor
            signal_config.enRot = 0;   // Disable rotation - keep IMU and optical data in sensor frame
            signal_config.enAcc = 1;   // Disable accelerometer data in Kalman filter
            signal_config.enVar = 1;   // Disable Kalman filter (set variance to 0) to get raw integrated position
            otosRight->setSignalProcessConfig(signal_config);
        }
    }
}

void odometrySetLowpassAlpha(float alpha)
{
    g_cfg.lowpass_alpha = alpha;
}

void odometryRequestScalar(float scalar)
{
    if (scalar < 0.7f) {
        scalar = 0.7f;
    } else if (scalar > 1.3f) {
        scalar = 1.3f;
    }
    g_scalar_left = scalar;
    g_scalar_right = scalar;
}

float odometryGetScalar()
{
    return g_scalar_left * 0.5f + g_scalar_right * 0.5f;
}

void odometryRequestAngularScalar(float scalar)
{
    otosLeft->setAngularScalar(scalar);
    otosRight->setAngularScalar(scalar);
}



void odometryCalibrateImu()
{
    if (otosLeft) {
        otosLeft->calibrateImu(255, false);
    }
    if (otosRight) {
        otosRight->calibrateImu(255, false);
    }
}

double averageAngle(double a1, double a2) {
    double x = cos(a1 * kDegToRad) + cos(a2 * kDegToRad);
    double y = sin(a1 * kDegToRad) + sin(a2 * kDegToRad);
    return atan2(y, x) * kRadToDeg;
}

void odometryUpdate(double dt)
{
    position_t pos_l, vel_l, acc_l;
    position_t pos_r, vel_r, acc_r;

    // Read position, velocity, and acceleration from both sensors
    bool left_ok = (otosLeft && otosLeft->getPosVelAcc(pos_l, vel_l, acc_l) == ret_OK);
    bool right_ok = (otosRight && otosRight->getPosVelAcc(pos_r, vel_r, acc_r) == ret_OK);

    if (!left_ok || !right_ok) {
        // Not enough sensors, cannot update
        is_odometry_ok = false;
        return;
    }
    
    // Sensors are responding
    is_odometry_ok = true;
    
    // Calculate delta positions for each sensor (change since last update)
    double dx_left = pos_l.x - g_prev_pos_left.x;
    double dy_left = pos_l.y - g_prev_pos_left.y;
    
    double dx_right = pos_r.x - g_prev_pos_right.x;
    double dy_right = pos_r.y - g_prev_pos_right.y;

    // Apply individual sensor scalars to correct for calibration differences
    dx_left *= g_scalar_left;
    dy_left *= g_scalar_left;
    dx_right *= g_scalar_right;
    dy_right *= g_scalar_right;

    // Both sensors measure the same robot motion, but from different mount points
    // Sensor geometry: left sensor at Y = -sensor_spacing/2, right sensor at Y = +sensor_spacing/2
    // Both sensors have X pointing forward, Y pointing left (in robot's local frame)
    
    // Forward displacement is average of both sensors' X displacement (in robot frame)
    double dx_robot_local = 0.5 * (dx_left + dx_right);
    
    // Lateral displacement is average of both sensors' Y displacement (in robot frame)
    double dy_robot_local = 0.5 * (dy_left + dy_right);
    
    // Angular displacement from difference in forward motion
    // When robot rotates, the two sensors move forward at different speeds
    // omega = (dx_right - dx_left) / sensor_spacing_mm
    double delta_angle_from_dx = 0.0;
    double d_dx = dx_right - dx_left;
    double omega_rad = d_dx / g_cfg.sensor_spacing_mm;  // Result is in radians
    delta_angle_from_dx = kRadToDeg * omega_rad;  // Convert to degrees

    double angle_imu = averageAngle(pos_l.a, pos_r.a);

    // Rotate local frame deltas to global frame using current heading angle
    double theta_rad = global_pos.a * kDegToRad;
    double cos_theta = cos(theta_rad);
    double sin_theta = sin(theta_rad);
    
    double dx_global = dx_robot_local * cos_theta - dy_robot_local * sin_theta;
    double dy_global = dx_robot_local * sin_theta + dy_robot_local * cos_theta;
    
    // Get angular velocity from IMU (average of both sensors)
    double omega_velocity = 0.5 * (vel_l.a + vel_r.a);  // This is in deg/s
    
    // Compute raw velocity from position deltas
    double vel_x_raw = dx_global / dt;
    double vel_y_raw = dy_global / dt;
    double vel_a_raw = omega_velocity * g_angular_scalar;

    double alpha = g_cfg.lowpass_alpha;
    // Apply low-pass filter to smooth velocity
    global_vel.x = alpha * global_vel.x + (1.0 - alpha) * vel_x_raw;
    global_vel.y = alpha * global_vel.y + (1.0 - alpha) * vel_y_raw;
    global_vel.a = vel_a_raw;
    
    // Auto-calibration: refine scalars during pure rotation
    double linear_vel_mag = sqrt(global_vel.x * global_vel.x + global_vel.y * global_vel.y);
    double angular_vel_mag = fabs(omega_velocity);

    double vel_x_left_raw = (pos_l.x - g_prev_pos_left.x) / dt;
    double vel_x_right_raw = (pos_r.x - g_prev_pos_right.x) / dt;
    double vel_y_left_raw = (pos_l.y - g_prev_pos_left.y) / dt;
    double vel_y_right_raw = (pos_r.y - g_prev_pos_right.y) / dt;
    filtered_vel_left.x = alpha * filtered_vel_left.x + (1.0 - alpha) * vel_x_left_raw;
    filtered_vel_right.x = alpha * filtered_vel_right.x + (1.0 - alpha) * vel_x_right_raw;
    filtered_vel_left.y = alpha * filtered_vel_left.y + (1.0 - alpha) * vel_y_left_raw;
    filtered_vel_right.y = alpha * filtered_vel_right.y + (1.0 - alpha) * vel_y_right_raw;
    
    // If the robot is rotating in place (low linear velocity, sufficient angular velocity), we can use the known geometry to estimate the correct scalars for each sensor
    if (linear_vel_mag < CALIB_MAX_LINEAR_VEL && angular_vel_mag > CALIB_MIN_ANGULAR_VEL) {
        // Robot is rotating in place - good for calibration
        
        // Convert omega_velocity (deg/s) to rad/s for calculation
        double omega_rad_from_vel = omega_velocity * kDegToRad;
        double sensor_radius = g_cfg.sensor_spacing_mm * 0.5;
        
        // Expected forward velocities from pure rotation
        double expected_vx_left = -omega_rad_from_vel * sensor_radius;
        double expected_vx_right = omega_rad_from_vel * sensor_radius;
        
        // Compute actual velocities from unscaled position deltas (before we applied scalars)
        // This isolates the optical sensor data from our correction factors

        // Calculate scalar samples from actual velocities
        if (fabs(vel_x_left_raw) > 1.0) {  // Avoid division by near-zero
            double sample_left = expected_vx_left / vel_x_left_raw;
            if (isfinite(sample_left) && sample_left > 0.1 && sample_left < 2.0) {
                g_calib_left_sum += sample_left;
                g_calib_left_count++;
            }
        }
        
        if (fabs(vel_x_right_raw) > 1.0) {  // Avoid division by near-zero
            double sample_right = expected_vx_right / vel_x_right_raw;
            if (isfinite(sample_right) && sample_right > 0.1 && sample_right < 2.0) {
                g_calib_right_sum += sample_right;
                g_calib_right_count++;
            }
        }
        
        // Check if we have enough calibration samples
        if (g_calib_left_count >= 10 && g_calib_right_count >= 10) {
            // Calculate average scalars (commented out for now)
            double new_left = g_calib_left_sum / g_calib_left_count;
            double new_right = g_calib_right_sum / g_calib_right_count;

            usartprintf(">sl:%.3f,sr:%.3f\r\n", new_left, new_right);
            
            // Reset calibration state
            g_calib_left_sum = 0.0;
            g_calib_right_sum = 0.0;
            g_calib_left_count = 0;
            g_calib_right_count = 0;
        }
    }
    
    // Update global position directly
    global_pos.x += dx_global;
    global_pos.y += dy_global;
    global_pos.a = angle_imu;  // Use gyro angle directly for better stability

    angle_from_dx += delta_angle_from_dx * g_angular_scalar;
    if (angle_from_dx > 180.0) {
        angle_from_dx -= 360.0;
    } else if (angle_from_dx < -180.0) {
        angle_from_dx += 360.0;
    }

    // Store debug info
    g_debug_imu_left_vel = vel_l;
    g_debug_imu_right_vel = vel_r;

    // Update previous positions for next iteration
    g_prev_pos_left = pos_l;
    g_prev_pos_right = pos_r;
}

void odometryResetPose(position_t pose)
{
    // Write to the otos the new pose
    otosLeft->setPosition(pose);
    otosRight->setPosition(pose);
    g_prev_pos_left = pose;
    g_prev_pos_right = pose;
    // Reset calibration state
    g_calib_left_sum = 0.0;
    g_calib_right_sum = 0.0;
    g_calib_left_count = 0;
    g_calib_right_count = 0;
}

void odometryDebugPrint()
{
    usartprintf(">x:%.2lf,y:%.2lf,a:%.2lf,vx:%.2lf,vy:%.2lf,va:%.2lf,a_dx:%.2lf,"
                "Limu_x:%.1lf,Limu_y:%.1lf,Rimuw_x:%.1lf,Rimu_y:%.1lf,"
                "Lraw_x:%.1lf,Lraw_y:%.1lf,Rraw_x:%.1lf,Rraw_y:%.1lf\r\n",
        global_pos.x, global_pos.y, global_pos.a,
        global_vel.x, global_vel.y, global_vel.a,
        angle_from_dx,
        g_debug_imu_left_vel.x, g_debug_imu_left_vel.y,
        g_debug_imu_right_vel.x, g_debug_imu_right_vel.y,
        filtered_vel_left.x, filtered_vel_left.y,
        filtered_vel_right.x, filtered_vel_right.y);
}