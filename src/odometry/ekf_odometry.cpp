#include "odometry/ekf_odometry.h"
#include "ctrl_math.h"
#include <math.h>

static constexpr double kDegToRad = PI / 180.0;

OdometryEkf::OdometryEkf()
{
    _q = ekf_process_noise_t();
    _r = ekf_measurement_noise_t();
    _gate_sigma = 3.5;
    reset({0.0, 0.0, 0.0});
}

void OdometryEkf::reset(const position_t &pose)
{
    _x[0] = pose.x;
    _x[1] = pose.y;
    _x[2] = pose.a;
    _x[3] = 0.0;
    _x[4] = 0.0;
    _x[5] = 0.0;

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            _p[i][j] = (i == j) ? 500.0 : 0.0;
        }
    }
}

void OdometryEkf::setProcessNoise(const ekf_process_noise_t &noise)
{
    _q = noise;
}

void OdometryEkf::setMeasurementNoise(const ekf_measurement_noise_t &noise)
{
    _r = noise;
}

void OdometryEkf::setGateSigma(double sigma)
{
    _gate_sigma = sigma;
}

void OdometryEkf::predict(double dt, double gyro_omega)
{
    _x[0] += _x[3] * dt;
    _x[1] += _x[4] * dt;
    _x[2] = mod_angle(_x[2] + _x[5] * dt);

    if (isfinite(gyro_omega)) {
        _x[5] = gyro_omega;
    }

    double f[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        f[i][i] = 1.0;
    }
    f[0][3] = dt;
    f[1][4] = dt;
    f[2][5] = dt;

    double fp[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0.0;
            for (int k = 0; k < 6; k++) {
                sum += f[i][k] * _p[k][j];
            }
            fp[i][j] = sum;
        }
    }

    double fpf_t[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0.0;
            for (int k = 0; k < 6; k++) {
                sum += fp[i][k] * f[j][k];
            }
            fpf_t[i][j] = sum;
        }
    }

    double q_pos = _q.pos_xy * dt * dt;
    double q_vel = _q.vel_xy * dt;
    double q_theta = _q.theta * dt * dt;
    double q_omega = _q.omega * dt;

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            _p[i][j] = fpf_t[i][j];
        }
    }

    _p[0][0] += q_pos;
    _p[1][1] += q_pos;
    _p[2][2] += q_theta;
    _p[3][3] += q_vel;
    _p[4][4] += q_vel;
    _p[5][5] += q_omega;
}

static bool invert5x5(const double in[5][5], double out[5][5])
{
    double a[5][10] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            a[i][j] = in[i][j];
        }
        a[i][i + 5] = 1.0;
    }

    for (int i = 0; i < 5; i++) {
        double pivot = a[i][i];
        if (fabs(pivot) < 1e-9) {
            return false;
        }
        double inv_pivot = 1.0 / pivot;
        for (int j = 0; j < 10; j++) {
            a[i][j] *= inv_pivot;
        }
        for (int r = 0; r < 5; r++) {
            if (r == i) {
                continue;
            }
            double factor = a[r][i];
            for (int c = 0; c < 10; c++) {
                a[r][c] -= factor * a[i][c];
            }
        }
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            out[i][j] = a[i][j + 5];
        }
    }

    return true;
}

void OdometryEkf::update(const double z[5], const double r_left[2], const double r_right[2], const bool valid[5], ekf_debug_t &dbg)
{
    double h[5][6] = {0};
    h[0][3] = 1.0;
    h[0][5] = -r_left[1] * kDegToRad;
    h[1][4] = 1.0;
    h[1][5] = r_left[0] * kDegToRad;
    h[2][3] = 1.0;
    h[2][5] = -r_right[1] * kDegToRad;
    h[3][4] = 1.0;
    h[3][5] = r_right[0] * kDegToRad;
    h[4][5] = 1.0;

    double hx[5] = {0};
    for (int i = 0; i < 5; i++) {
        double sum = 0.0;
        for (int j = 0; j < 6; j++) {
            sum += h[i][j] * _x[j];
        }
        hx[i] = sum;
    }

    double y[5] = {0};
    for (int i = 0; i < 5; i++) {
        y[i] = z[i] - hx[i];
    }

    double r_diag[5] = {_r.otos_vx, _r.otos_vy, _r.otos_vx, _r.otos_vy, _r.imu_omega};

    double pht[6][5] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 5; j++) {
            double sum = 0.0;
            for (int k = 0; k < 6; k++) {
                sum += _p[i][k] * h[j][k];
            }
            pht[i][j] = sum;
        }
    }

    double s[5][5] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double sum = 0.0;
            for (int k = 0; k < 6; k++) {
                sum += h[i][k] * pht[k][j];
            }
            s[i][j] = sum;
        }
    }

    for (int i = 0; i < 5; i++) {
        s[i][i] += r_diag[i];
    }

    bool use[5] = {valid[0], valid[1], valid[2], valid[3], valid[4]};
    for (int i = 0; i < 5; i++) {
        double sigma = sqrt(fmax(1e-9, s[i][i]));
        dbg.innovation[i] = y[i];
        dbg.innovation_sigma[i] = sigma;
        if (!use[i] || fabs(y[i]) > _gate_sigma * sigma) {
            use[i] = false;
        }
    }

    bool rejected = false;
    for (int i = 0; i < 5; i++) {
        if (!use[i]) {
            s[i][i] += 1e9;
            rejected = true;
        }
    }

    double s_inv[5][5];
    if (!invert5x5(s, s_inv)) {
        return;
    }

    double k_gain[6][5] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 5; j++) {
            double sum = 0.0;
            for (int k = 0; k < 5; k++) {
                sum += pht[i][k] * s_inv[k][j];
            }
            k_gain[i][j] = sum;
        }
    }

    for (int i = 0; i < 6; i++) {
        double sum = 0.0;
        for (int j = 0; j < 5; j++) {
            sum += k_gain[i][j] * y[j];
        }
        _x[i] += sum;
    }
    _x[2] = mod_angle(_x[2]);

    double kh[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0.0;
            for (int k = 0; k < 5; k++) {
                sum += k_gain[i][k] * h[k][j];
            }
            kh[i][j] = sum;
        }
    }

    double i_kh[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            i_kh[i][j] = (i == j) ? 1.0 - kh[i][j] : -kh[i][j];
        }
    }

    double new_p[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0.0;
            for (int k = 0; k < 6; k++) {
                sum += i_kh[i][k] * _p[k][j];
            }
            new_p[i][j] = sum;
        }
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            _p[i][j] = new_p[i][j];
        }
    }
}

void OdometryEkf::getState(position_t &pos, position_t &vel) const
{
    pos.x = _x[0];
    pos.y = _x[1];
    pos.a = _x[2];
    vel.x = _x[3];
    vel.y = _x[4];
    vel.a = _x[5];
}

void OdometryEkf::getCovarianceDiag(double diag[6]) const
{
    for (int i = 0; i < 6; i++) {
        diag[i] = _p[i][i];
    }
}
