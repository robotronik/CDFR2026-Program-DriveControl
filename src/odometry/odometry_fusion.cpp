#include "odometry/odometry_fusion.h"
#include "ctrl_math.h"
#include "clock.h"
#include <math.h>

static constexpr double kDegToRad = PI / 180.0;
static constexpr double kRadToDeg = 180.0 / PI;

static DualOtos g_dual;
static odometry_config_t g_cfg;
static odometry_debug_t g_dbg;

// State: position and velocity
static position_t g_pos = {0.0, 0.0, 0.0};
static position_t g_vel = {0.0, 0.0, 0.0};
static position_t g_vel_filtered = {0.0, 0.0, 0.0};
static position_t g_offset = {0.0, 0.0, 0.0};

// Previous sensor positions for delta calculation
static position_t g_prev_pos_left = {0.0, 0.0, 0.0};
static position_t g_prev_pos_right = {0.0, 0.0, 0.0};

static float g_scalar = 1.0f;
static float g_scalar_left = 1.0f;
static float g_scalar_right = 1.0f;
static float g_angular_scalar = 1.0f;

// Auto-calibration during rotation
static double g_calib_left_sum = 0.0;
static double g_calib_right_sum = 0.0;
static int g_calib_left_count = 0;
static int g_calib_right_count = 0;
static constexpr double CALIB_MIN_LINEAR_VEL = 10.0;   // mm/s
static constexpr double CALIB_MIN_ANGULAR_VEL = 5.0;  // deg/s

void odometrySetup(const odometry_config_t &config)
{
    g_cfg = config;

    dual_otos_config_t otos_cfg;
    otos_cfg.lowpass_alpha = config.lowpass_alpha;
    otos_cfg.address_left = config.address_left;
    otos_cfg.address_right = config.address_right;

    g_dual.begin(otos_cfg);

    otosLeft = g_dual.left();
    otosRight = g_dual.right();
    i2cDeviceLeft = g_dual.leftBus();
    i2cDeviceRight = g_dual.rightBus();

    // Configure signal processing: disable rotation of IMU/optical data by heading
    if (otosLeft) {
        otos_signal_process_config_t signal_config;
        if (otosLeft->getSignalProcessConfig(signal_config) == ret_OK) {
            signal_config.enRot = 0;  // Disable rotation - keep IMU and optical data in sensor frame
            otosLeft->setSignalProcessConfig(signal_config);
        }
    }
    if (otosRight) {
        otos_signal_process_config_t signal_config;
        if (otosRight->getSignalProcessConfig(signal_config) == ret_OK) {
            signal_config.enRot = 0;  // Disable rotation - keep IMU and optical data in sensor frame
            otosRight->setSignalProcessConfig(signal_config);
        }
    }
}

void odometrySetLowpassAlpha(float alpha)
{
    g_cfg.lowpass_alpha = alpha;
    dual_otos_config_t otos_cfg;
    otos_cfg.address_left = i2cDeviceLeft ? i2cDeviceLeft->address() : kDefaultAddress;
    otos_cfg.address_right = i2cDeviceRight ? i2cDeviceRight->address() : (uint8_t)(kDefaultAddress + 1);
    otos_cfg.lowpass_alpha = alpha;
    g_dual.begin(otos_cfg);
}

void odometryRequestScalar(float scalar)
{
    if (scalar < 0.7f) {
        scalar = 0.7f;
    } else if (scalar > 1.3f) {
        scalar = 1.3f;
    }
    g_scalar = scalar;
    g_scalar_left = scalar;
    g_scalar_right = scalar;
}

float odometryGetScalar()
{
    return g_scalar;
}

void odometryRequestAngularScalar(float scalar)
{
    if (scalar < 0.7f) {
        scalar = 0.7f;
    } else if (scalar > 1.3f) {
        scalar = 1.3f;
    }
    g_angular_scalar = scalar;
}

float odometryGetAngularScalar()
{
    return g_angular_scalar;
}

void odometryRequestOffset(const position_t &offset)
{
    g_offset = offset;
}

position_t odometryGetOffset()
{
    return g_offset;
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

#include "uart.h"

void odometryUpdate(double dt)
{
    position_t pos_l, vel_l, acc_l;
    position_t pos_r, vel_r, acc_r;

    // Read position, velocity, and acceleration from both sensors
    bool left_ok = (otosLeft && otosLeft->getPosVelAcc(pos_l, vel_l, acc_l) == ret_OK);
    bool right_ok = (otosRight && otosRight->getPosVelAcc(pos_r, vel_r, acc_r) == ret_OK);

    g_dbg.left_ok = left_ok;
    g_dbg.right_ok = right_ok;

    if (!left_ok || !right_ok) {
        // Not enough sensors, cannot update
        return;
    }

    // Store raw sensor velocities for debugging
    g_dbg.raw_left_vel = vel_l;
    g_dbg.raw_right_vel = vel_r;
    
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

    double angle_gyro = 0.5 * (pos_l.a + pos_r.a);

    // Rotate local frame deltas to global frame using current heading angle
    double theta_rad = g_pos.a * kDegToRad;
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
    g_vel_filtered.x = alpha * g_vel_filtered.x + (1.0 - alpha) * vel_x_raw;
    g_vel_filtered.y = alpha * g_vel_filtered.y + (1.0 - alpha) * vel_y_raw;
    g_vel_filtered.a = vel_a_raw;
    
    // Use filtered velocity for output
    g_vel = g_vel_filtered;
    
    // Auto-calibration: refine scalars during pure rotation
    double linear_vel_mag = sqrt(g_vel.x * g_vel.x + g_vel.y * g_vel.y);
    double angular_vel_mag = fabs(omega_velocity);

    static double actual_vel_x_left = 0;
    static double actual_vel_x_right = 0;
    double vel_x_left_raw = (pos_l.x - g_prev_pos_left.x) / dt;
    double vel_x_right_raw = (pos_r.x - g_prev_pos_right.x) / dt;
    actual_vel_x_left = alpha * actual_vel_x_left + (1.0 - alpha) * vel_x_left_raw;
    actual_vel_x_right = alpha * actual_vel_x_right + (1.0 - alpha) * vel_x_right_raw;
    
    // If the robot is rotating in place (low linear velocity, sufficient angular velocity), we can use the known geometry to estimate the correct scalars for each sensor
    if (linear_vel_mag < CALIB_MIN_LINEAR_VEL && angular_vel_mag > CALIB_MIN_ANGULAR_VEL) {
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
        if (fabs(actual_vel_x_left) > 1.0) {  // Avoid division by near-zero
            double sample_left = expected_vx_left / actual_vel_x_left;
            if (isfinite(sample_left) && sample_left > 0.1 && sample_left < 2.0) {
                g_calib_left_sum += sample_left;
                g_calib_left_count++;
            }
        }
        
        if (fabs(actual_vel_x_right) > 1.0) {  // Avoid division by near-zero
            double sample_right = expected_vx_right / actual_vel_x_right;
            if (isfinite(sample_right) && sample_right > 0.1 && sample_right < 2.0) {
                g_calib_right_sum += sample_right;
                g_calib_right_count++;
            }
        }
        
        // Check if we have enough calibration samples
        if (g_calib_left_count >= 50 && g_calib_right_count >= 50) {
            // Calculate average scalars
            double new_left = g_calib_left_sum / g_calib_left_count;
            double new_right = g_calib_right_sum / g_calib_right_count;
            //g_scalar_left = new_left * 0.3 + g_scalar_left * 0.7;
            //g_scalar_right = new_right * 0.3 + g_scalar_right * 0.7;
            
            // Reset calibration state
            g_calib_left_sum = 0.0;
            g_calib_right_sum = 0.0;
            g_calib_left_count = 0;
            g_calib_right_count = 0;
        }
    }
    
    g_pos.x += dx_global;
    g_pos.y += dy_global;
    //g_pos.a = mod_angle(g_pos.a + delta_angle_from_dx * g_angular_scalar);
    g_pos.a = angle_gyro;  // Use gyro angle directly for better stability

    static double angle_from_dx = 0.0;
    angle_from_dx += delta_angle_from_dx * g_angular_scalar;

    // Store debug info
    g_dbg.filt_left_vel = vel_l;
    g_dbg.filt_right_vel = vel_r;
    g_dbg.merged_vel.x = g_vel.x;
    g_dbg.merged_vel.y = g_vel.y;
    g_dbg.merged_vel.a = g_vel.a;
    g_dbg.angle_dx = angle_from_dx;
    g_dbg.angle_gyro = angle_gyro;
    g_dbg.omega_merged = g_vel.a;
    g_dbg.scalar = g_scalar;
    g_dbg.angular_scalar = g_angular_scalar;
    g_dbg.scalar_left = g_scalar_left;
    g_dbg.scalar_right = g_scalar_right;

    // Update previous positions for next iteration
    g_prev_pos_left = pos_l;
    g_prev_pos_right = pos_r;
}

void odometryResetPose(const position_t &pose)
{
    g_pos = pose;
    g_vel = {0.0, 0.0, 0.0};
    g_vel_filtered = {0.0, 0.0, 0.0};
    // Reset calibration state
    g_calib_left_sum = 0.0;
    g_calib_right_sum = 0.0;
    g_calib_left_count = 0;
    g_calib_right_count = 0;
}

void odometryGetPose(position_t &pos, position_t &vel)
{
    pos = g_pos;
    pos.x += g_offset.x;
    pos.y += g_offset.y;
    pos.a = mod_angle(pos.a + g_offset.a);
    
    vel = g_vel;
}

const odometry_debug_t &odometryDebug()
{
    return g_dbg;
}
