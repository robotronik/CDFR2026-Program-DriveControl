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
static position_t g_offset = {0.0, 0.0, 0.0};

static float g_scalar = 1.0f;
static float g_scalar_left = 1.0f;
static float g_scalar_right = 1.0f;
static bool g_scalar_pending = false;
static float g_angular_scalar = 1.0f;

static bool g_calib_active = false;
static uint32_t g_calib_start_us = 0;
static int g_calib_count = 0;
static double g_calib_sum = 0.0;

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
    g_scalar_pending = true;
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

void odometryStartScalarCalibration()
{
    g_calib_active = true;
    g_calib_start_us = micros();
    g_calib_count = 0;
    g_calib_sum = 0.0;
}

bool odometryIsScalarCalibrating()
{
    return g_calib_active;
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

static void update_scalar_calibration(double omega_computed, const otos_sample_t &left, const otos_sample_t &right)
{
    const double min_omega = 5.0;
    const double min_vel = 5.0;

    if (!g_calib_active || !left.ok || !right.ok) {
        return;
    }

    if (fabs(omega_computed) < min_omega) {
        return;
    }

    const double omega_rad = omega_computed * kDegToRad;
    const double r_left_y = -g_cfg.sensor_spacing_mm * 0.5;
    const double r_right_y = g_cfg.sensor_spacing_mm * 0.5;

    if (fabs(left.vel_filtered.x) > min_vel) {
        double sample = (omega_rad * r_left_y) / left.vel_filtered.x;
        if (isfinite(sample)) {
            g_calib_sum += fabs(sample);
            g_calib_count++;
        }
    }

    if (fabs(right.vel_filtered.x) > min_vel) {
        double sample = (omega_rad * r_right_y) / right.vel_filtered.x;
        if (isfinite(sample)) {
            g_calib_sum += fabs(sample);
            g_calib_count++;
        }
    }

    uint32_t elapsed = micros() - g_calib_start_us;
    if (g_calib_count >= 200 || elapsed > 2000000) {
        if (g_calib_count > 0) {
            odometryRequestScalar((float)(g_calib_sum / g_calib_count));
        }
        g_calib_active = false;
    }
}

void odometryUpdate(double dt)
{
    otos_sample_t left_sample;
    otos_sample_t right_sample;

    g_dual.poll(left_sample, right_sample);

    g_dbg.left_ok = left_sample.ok;
    g_dbg.right_ok = right_sample.ok;
    g_dbg.raw_left_vel = left_sample.ok ? left_sample.vel : position_t{0.0, 0.0, 0.0};
    g_dbg.raw_right_vel = right_sample.ok ? right_sample.vel : position_t{0.0, 0.0, 0.0};
    g_dbg.filt_left_vel = left_sample.ok ? left_sample.vel_filtered : position_t{0.0, 0.0, 0.0};
    g_dbg.filt_right_vel = right_sample.ok ? right_sample.vel_filtered : position_t{0.0, 0.0, 0.0};

    if (g_scalar_pending) {
        g_scalar_pending = false;
    }

    position_t left_vel = left_sample.ok ? left_sample.vel_filtered : position_t{0.0, 0.0, 0.0};
    position_t right_vel = right_sample.ok ? right_sample.vel_filtered : position_t{0.0, 0.0, 0.0};

    left_vel.x *= g_scalar_left;
    left_vel.y *= g_scalar_left;
    right_vel.x *= g_scalar_right;
    right_vel.y *= g_scalar_right;

    // Compute omega from dual sensor velocity difference
    double omega_computed = 0.0;
    if (left_sample.ok && right_sample.ok && g_cfg.sensor_spacing_mm > 1.0f) {
        double omega_rad = (right_vel.x - left_vel.x) / g_cfg.sensor_spacing_mm;
        omega_computed = omega_rad * kRadToDeg;
    }

    // Get gyro readings from both sensors and average them
    double omega_gyro = 0.0;
    int gyro_count = 0;
    if (left_sample.ok && isfinite(left_vel.a)) {
        omega_gyro += left_vel.a;
        gyro_count++;
    }
    if (right_sample.ok && isfinite(right_vel.a)) {
        omega_gyro += right_vel.a;
        gyro_count++;
    }
    if (gyro_count > 0) {
        omega_gyro /= gyro_count;
    } else {
        omega_gyro = NAN;
    }

    // Merge omega sources: prefer velocity-computed when both sensors are moving,
    // fall back to gyro readings otherwise
    double omega_merged = NAN;
    if (isfinite(omega_computed) && left_sample.ok && right_sample.ok && g_cfg.sensor_spacing_mm > 1.0f) {
        // Both velocity difference and gyro available: use weighted average
        // Weight velocity-computed omega higher when sensors are moving
        bool moving = (fabs(left_vel.x) > 5.0 || fabs(left_vel.y) > 5.0 ||
                       fabs(right_vel.x) > 5.0 || fabs(right_vel.y) > 5.0);
        if (moving && isfinite(omega_gyro)) {
            // Moving: blend both sources (velocity 0.6, gyro 0.4)
            omega_merged = 0.6 * omega_computed + 0.4 * omega_gyro;
        } else if (moving) {
            // Moving but no gyro: use velocity-computed
            omega_merged = omega_computed;
        } else if (isfinite(omega_gyro)) {
            // Stationary: prefer gyro as velocity difference becomes unreliable
            omega_merged = omega_gyro;
        } else {
            // Stationary with no gyro: use velocity-computed
            omega_merged = omega_computed;
        }
    } else if (isfinite(omega_gyro)) {
        // Only gyro available: use it
        omega_merged = omega_gyro;
    } else if (isfinite(omega_computed)) {
        // Only velocity-computed available: use it
        omega_merged = omega_computed;
    }

    // Apply angular scalar to merged omega
    if (isfinite(omega_merged)) {
        omega_merged *= g_angular_scalar;
    }

    // Zero-velocity constraint: if both sensors report very low velocities, 
    // force velocity measurements to zero to prevent drift
    const double zero_vel_threshold = 10.0;  // mm/s
    bool left_stationary = fabs(left_vel.x) < zero_vel_threshold && fabs(left_vel.y) < zero_vel_threshold;
    bool right_stationary = fabs(right_vel.x) < zero_vel_threshold && fabs(right_vel.y) < zero_vel_threshold;
    
    if (left_stationary && right_stationary && left_sample.ok && right_sample.ok) {
        // Both sensors agree robot is stationary - force velocity to zero
        left_vel.x = 0.0;
        left_vel.y = 0.0;
        right_vel.x = 0.0;
        right_vel.y = 0.0;
        omega_merged = 0.0;
    }

    // Merge sensors: average velocities from both sensors
    position_t merged_vel = {0.0, 0.0, 0.0};
    if (left_sample.ok && right_sample.ok) {
        merged_vel.x = 0.5 * (left_vel.x + right_vel.x);
        merged_vel.y = 0.5 * (left_vel.y + right_vel.y);
        merged_vel.a = omega_merged;
    } else if (left_sample.ok) {
        merged_vel = left_vel;
        merged_vel.a = omega_merged;
    } else if (right_sample.ok) {
        merged_vel = right_vel;
        merged_vel.a = omega_merged;
    }

    g_dbg.merged_vel = merged_vel;
    g_dbg.omega_computed = omega_computed;
    g_dbg.omega_gyro = omega_gyro;
    g_dbg.omega_merged = omega_merged;
    g_dbg.scalar = g_scalar;
    g_dbg.angular_scalar = g_angular_scalar;

    update_scalar_calibration(omega_computed, left_sample, right_sample);

    // Simple dead reckoning: integrate velocity and angle
    g_vel = merged_vel;

    g_pos.x += merged_vel.x * dt;
    g_pos.y += merged_vel.y * dt;
    g_pos.a = mod_angle(g_pos.a + merged_vel.a * dt);
}

void odometryResetPose(const position_t &pose)
{
    g_pos = pose;
    g_vel = {0.0, 0.0, 0.0};
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
