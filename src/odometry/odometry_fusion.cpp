#include "odometry/odometry_fusion.h"
#include "ctrl_math.h"
#include "clock.h"
#include <math.h>

static constexpr double kDegToRad = PI / 180.0;
static constexpr double kRadToDeg = 180.0 / PI;

static DualOtos g_dual;
static OdometryEkf g_ekf;
static odometry_config_t g_cfg;
static odometry_debug_t g_dbg;
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

    g_ekf.setGateSigma(config.gate_sigma);
}

void odometrySetProcessNoise(const ekf_process_noise_t &noise)
{
    g_ekf.setProcessNoise(noise);
}

void odometrySetMeasurementNoise(const ekf_measurement_noise_t &noise)
{
    g_ekf.setMeasurementNoise(noise);
}

void odometrySetGateSigma(double sigma)
{
    g_cfg.gate_sigma = sigma;
    g_ekf.setGateSigma(sigma);
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

static void update_scalar_calibration(double omega_imu, const otos_sample_t &left, const otos_sample_t &right)
{
    const double min_omega = 5.0;
    const double min_vel = 5.0;

    if (!g_calib_active || !left.ok || !right.ok) {
        return;
    }

    if (fabs(omega_imu) < min_omega) {
        return;
    }

    const double omega_rad = omega_imu * kDegToRad;
    const double r_left_x = -g_cfg.sensor_spacing_mm * 0.5;
    const double r_right_x = g_cfg.sensor_spacing_mm * 0.5;

    if (fabs(left.vel_filtered.y) > min_vel) {
        double sample = (omega_rad * r_left_x) / left.vel_filtered.y;
        if (isfinite(sample)) {
            g_calib_sum += fabs(sample);
            g_calib_count++;
        }
    }

    if (fabs(right.vel_filtered.y) > min_vel) {
        double sample = (omega_rad * r_right_x) / right.vel_filtered.y;
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

    double omega_imu = NAN;
    if (left_sample.ok && right_sample.ok) {
        omega_imu = 0.5 * (left_vel.a + right_vel.a);
    } else if (left_sample.ok) {
        omega_imu = left_vel.a;
    } else if (right_sample.ok) {
        omega_imu = right_vel.a;
    }

    if (isfinite(omega_imu)) {
        omega_imu *= g_angular_scalar;
    }

    double omega_otos = 0.0;
    position_t rigid_vel = {0.0, 0.0, 0.0};
    if (left_sample.ok && right_sample.ok && g_cfg.sensor_spacing_mm > 1.0f) {
        double omega_rad = (right_vel.y - left_vel.y) / g_cfg.sensor_spacing_mm;
        omega_otos = omega_rad * kRadToDeg;
        rigid_vel.x = 0.5 * (left_vel.x + right_vel.x);
        rigid_vel.y = 0.5 * (left_vel.y + right_vel.y);
        rigid_vel.a = omega_otos;
    }

    g_dbg.omega_otos = omega_otos;
    g_dbg.omega_imu = omega_imu;
    g_dbg.scalar = g_scalar;
    g_dbg.angular_scalar = g_angular_scalar;

    update_scalar_calibration(omega_imu, left_sample, right_sample);

    g_ekf.predict(dt, omega_imu);

    double z[5] = {left_vel.x, left_vel.y, right_vel.x, right_vel.y, omega_imu};
    bool valid[5] = {left_sample.ok, left_sample.ok, right_sample.ok, right_sample.ok, isfinite(omega_imu)};

    double r_left[2] = {-g_cfg.sensor_spacing_mm * 0.5, 0.0};
    double r_right[2] = {g_cfg.sensor_spacing_mm * 0.5, 0.0};

    ekf_debug_t ekf_dbg;
    g_ekf.update(z, r_left, r_right, valid, ekf_dbg);
    for (int i = 0; i < 5; i++) {
        g_dbg.innovation[i] = ekf_dbg.innovation[i];
        g_dbg.innovation_sigma[i] = ekf_dbg.innovation_sigma[i];
    }

    position_t pos;
    position_t vel;
    g_ekf.getState(pos, vel);

    pos.x += g_offset.x;
    pos.y += g_offset.y;
    pos.a = mod_angle(pos.a + g_offset.a);

    g_dbg.fused_vel = rigid_vel;
    g_ekf.getCovarianceDiag(g_dbg.cov_diag);
}

void odometryResetPose(const position_t &pose)
{
    g_ekf.reset(pose);
}

void odometryGetPose(position_t &pos, position_t &vel)
{
    g_ekf.getState(pos, vel);
    pos.x += g_offset.x;
    pos.y += g_offset.y;
    pos.a = mod_angle(pos.a + g_offset.a);
}

const odometry_debug_t &odometryDebug()
{
    return g_dbg;
}
