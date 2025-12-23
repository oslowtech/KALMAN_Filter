#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float x[2];
    float P[2][2];
    float Q[2][2];
    float R;
    float dt;
} KalmanFilter;

void kalman_init(KalmanFilter *kf, float dt, float q_pos, float q_vel, float r_meas);
void kalman_predict(KalmanFilter *kf, float acceleration);
void kalman_update(KalmanFilter *kf, float measurement);
float kalman_get_altitude(KalmanFilter *kf);
float kalman_get_velocity(KalmanFilter *kf);

#endif
