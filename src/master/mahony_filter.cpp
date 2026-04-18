#include "mahony_filter.h"

#include <cmath>

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEG2RAD = PI_F / 180.0f;
}

MahonyFilter::MahonyFilter()
    : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
      kp(2.0f), ki(0.0f),
      exInt(0.0f), eyInt(0.0f), ezInt(0.0f) {}

void MahonyFilter::reset() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
}

void MahonyFilter::setGains(float kpIn, float kiIn) {
    kp = kpIn;
    ki = kiIn;
}

void MahonyFilter::update(
    float ax,
    float ay,
    float az,
    float gxDps,
    float gyDps,
    float gzDps,
    float mx,
    float my,
    float mz,
    float dt,
    bool useGyro,
    bool useMag
) {
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;

    float gx = gxDps * DEG2RAD;
    float gy = gyDps * DEG2RAD;
    float gz = gzDps * DEG2RAD;

    if (!useGyro) {
        float normAcc = std::sqrt(ax * ax + ay * ay + az * az);
        if (normAcc > 1e-6f) {
            float rollAcc = std::atan2(ay, az);
            float pitchAcc = std::atan2(-ax, std::sqrt(ay * ay + az * az));

            float yawMag = 0.0f;
            if (useMag) {
                float normMag = std::sqrt(mx * mx + my * my + mz * mz);
                if (normMag > 1e-6f) {
                    float mxn = mx / normMag;
                    float myn = my / normMag;
                    float mzn = mz / normMag;
                    float cr1 = std::cos(rollAcc);
                    float sr1 = std::sin(rollAcc);
                    float cp1 = std::cos(pitchAcc);
                    float sp1 = std::sin(pitchAcc);
                    float mxh = mxn * cp1 + myn * sp1 * sr1 + mzn * sp1 * cr1;
                    float myh = myn * cr1 - mzn * sr1;
                    yawMag = std::atan2(myh, mxh);
                }
            }

            float cr = std::cos(rollAcc * 0.5f);
            float sr = std::sin(rollAcc * 0.5f);
            float cp = std::cos(pitchAcc * 0.5f);
            float sp = std::sin(pitchAcc * 0.5f);
            float cy = std::cos(yawMag * 0.5f);
            float sy = std::sin(yawMag * 0.5f);

            q0 = cr * cp * cy + sr * sp * sy;
            q1 = sr * cp * cy - cr * sp * sy;
            q2 = cr * sp * cy + sr * cp * sy;
            q3 = cr * cp * sy - sr * sp * cy;

            exInt = 0.0f;
            eyInt = 0.0f;
            ezInt = 0.0f;
        }
        return;
    }

    float qa = q0;
    float qb = q1;
    float qc = q2;
    float qd = q3;

    float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        ax /= norm;
        ay /= norm;
        az /= norm;

        float vx = 2.0f * (qb * qd - qa * qc);
        float vy = 2.0f * (qa * qb + qc * qd);
        float vz = qa * qa - qb * qb - qc * qc + qd * qd;

        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        if (useMag) {
            float magNorm = std::sqrt(mx * mx + my * my + mz * mz);
            if (magNorm > 1e-6f) {
                mx /= magNorm;
                my /= magNorm;
                mz /= magNorm;

                float hx = 2.0f * (mx * (0.5f - qc * qc - qd * qd) + my * (qb * qc - qa * qd) + mz * (qb * qd + qa * qc));
                float hy = 2.0f * (mx * (qb * qc + qa * qd) + my * (0.5f - qb * qb - qd * qd) + mz * (qc * qd - qa * qb));
                float bx = std::sqrt(hx * hx + hy * hy);
                float bz = 2.0f * (mx * (qb * qd - qa * qc) + my * (qc * qd + qa * qb) + mz * (0.5f - qb * qb - qc * qc));

                float wx = 2.0f * (bx * (0.5f - qc * qc - qd * qd) + bz * (qb * qd - qa * qc));
                float wy = 2.0f * (bx * (qb * qc - qa * qd) + bz * (qa * qb + qc * qd));
                float wz = 2.0f * (bx * (qa * qc + qb * qd) + bz * (0.5f - qb * qb - qc * qc));

                ex += (my * wz - mz * wy);
                ey += (mz * wx - mx * wz);
                ez += (mx * wy - my * wx);
            }
        }

        exInt += ex * ki * dt;
        eyInt += ey * ki * dt;
        ezInt += ez * ki * dt;

        gx += kp * ex + exInt;
        gy += kp * ey + eyInt;
        gz += kp * ez + ezInt;
    }

    q0 += (-qb * gx - qc * gy - qd * gz) * (0.5f * dt);
    q1 += (qa * gx + qc * gz - qd * gy) * (0.5f * dt);
    q2 += (qa * gy - qb * gz + qd * gx) * (0.5f * dt);
    q3 += (qa * gz + qb * gy - qc * gx) * (0.5f * dt);

    float qNorm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qNorm > 0.0f) {
        q0 /= qNorm;
        q1 /= qNorm;
        q2 /= qNorm;
        q3 /= qNorm;
    }
}

void MahonyFilter::getQuaternion(float *qw, float *qx, float *qy, float *qz) const {
    if (qw) *qw = q0;
    if (qx) *qx = q1;
    if (qy) *qy = q2;
    if (qz) *qz = q3;
}
