#pragma once

class MahonyFilter {
public:
    MahonyFilter();

    void reset();
    void setGains(float kp, float ki);

    void update(
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
        bool useGyro = true,
        bool useMag = true
    );

    void getQuaternion(float *qw, float *qx, float *qy, float *qz) const;

private:
    float q0;
    float q1;
    float q2;
    float q3;

    float kp;
    float ki;

    float exInt;
    float eyInt;
    float ezInt;
};
