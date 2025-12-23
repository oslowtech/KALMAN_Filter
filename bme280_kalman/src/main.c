#include <stdio.h>
#include "bme280.h"
#include "kalman.h"

#define DT 0.1f
#define Q_POS 0.01f
#define Q_VEL 0.1f
#define R_MEAS 9.0f

BME280 sensor;
KalmanFilter kf;

void delay_ms(int ms);

void setup(void) {
    if (bme280_init(&sensor) != 0) {
        printf("BME280 init failed\n");
        while (1);
    }
    
    kalman_init(&kf, DT, Q_POS, Q_VEL, R_MEAS);
    
    int32_t raw_t, raw_p, raw_h;
    bme280_read_raw(&sensor, &raw_t, &raw_p, &raw_h);
    bme280_compensate_temp(&sensor, raw_t);
    float pressure = bme280_compensate_press(&sensor, raw_p);
    bme280_set_sea_level(&sensor, pressure);
}

void loop(void) {
    int32_t raw_t, raw_p, raw_h;
    
    if (bme280_read_raw(&sensor, &raw_t, &raw_p, &raw_h) != 0) {
        return;
    }
    
    float temp = bme280_compensate_temp(&sensor, raw_t);
    float pressure = bme280_compensate_press(&sensor, raw_p);
    float humidity = bme280_compensate_hum(&sensor, raw_h);
    
    float raw_alt = bme280_calc_altitude(&sensor, pressure);
    
    kalman_predict(&kf, 0.0f);
    kalman_update(&kf, raw_alt);
    
    float filtered_alt = kalman_get_altitude(&kf);
    float velocity = kalman_get_velocity(&kf);
    
    printf("T:%.2f P:%.2f H:%.2f ALT:%.2f FALT:%.2f V:%.2f\n",
           temp, pressure / 100.0f, humidity, raw_alt, filtered_alt, velocity);
    
    delay_ms((int)(DT * 1000));
}

int main(void) {
    setup();
    
    while (1) {
        loop();
    }
    
    return 0;
}

__attribute__((weak)) void delay_ms(int ms) {
    for (volatile int i = 0; i < ms * 1000; i++);
}

__attribute__((weak)) int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    (void)addr;
    (void)reg;
    (void)val;
    return 0;
}

__attribute__((weak)) int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    (void)addr;
    (void)reg;
    (void)buf;
    (void)len;
    return 0;
}
