#include "kalman.h"

void kalman_init(KalmanFilter *kf, float dt, float q_pos, float q_vel, float r_meas) {
    kf->dt = dt;
    
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    
    kf->Q[0][0] = q_pos;
    kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;
    kf->Q[1][1] = q_vel;
    
    kf->R = r_meas;
}

void kalman_predict(KalmanFilter *kf, float acceleration) {
    float dt = kf->dt;
    float dt2 = 0.5f * dt * dt;
    
    float x0 = kf->x[0];
    float x1 = kf->x[1];
    
    kf->x[0] = x0 + dt * x1 + dt2 * acceleration;
    kf->x[1] = x1 + dt * acceleration;
    
    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    float p10 = kf->P[1][0];
    float p11 = kf->P[1][1];
    
    kf->P[0][0] = p00 + dt * p10 + dt * (p01 + dt * p11) + kf->Q[0][0];
    kf->P[0][1] = p01 + dt * p11;
    kf->P[1][0] = p10 + dt * p11;
    kf->P[1][1] = p11 + kf->Q[1][1];
}

void kalman_update(KalmanFilter *kf, float measurement) {
    float y = measurement - kf->x[0];
    
    float s = kf->P[0][0] + kf->R;
    
    float k0 = kf->P[0][0] / s;
    float k1 = kf->P[1][0] / s;
    
    kf->x[0] = kf->x[0] + k0 * y;
    kf->x[1] = kf->x[1] + k1 * y;
    
    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    
    kf->P[0][0] = p00 - k0 * p00;
    kf->P[0][1] = p01 - k0 * p01;
    kf->P[1][0] = kf->P[1][0] - k1 * p00;
    kf->P[1][1] = kf->P[1][1] - k1 * p01;
}

float kalman_get_altitude(KalmanFilter *kf) {
    return kf->x[0];
}

float kalman_get_velocity(KalmanFilter *kf) {
    return kf->x[1];
}
