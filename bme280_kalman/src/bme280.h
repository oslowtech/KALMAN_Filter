#ifndef BME280_H
#define BME280_H

#include <stdint.h>

#define BME280_ADDR 0x76

#define BME280_REG_ID           0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_REG_PRESS_LSB    0xF8
#define BME280_REG_PRESS_XLSB   0xF9
#define BME280_REG_TEMP_MSB     0xFA
#define BME280_REG_TEMP_LSB     0xFB
#define BME280_REG_TEMP_XLSB    0xFC
#define BME280_REG_HUM_MSB      0xFD
#define BME280_REG_HUM_LSB      0xFE

#define BME280_REG_CALIB_00     0x88
#define BME280_REG_CALIB_26     0xE1

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_Calib;

typedef struct {
    BME280_Calib calib;
    int32_t t_fine;
    float sea_level_pressure;
} BME280;

int bme280_init(BME280 *dev);
int bme280_read_raw(BME280 *dev, int32_t *temp, int32_t *press, int32_t *hum);
float bme280_compensate_temp(BME280 *dev, int32_t raw_temp);
float bme280_compensate_press(BME280 *dev, int32_t raw_press);
float bme280_compensate_hum(BME280 *dev, int32_t raw_hum);
float bme280_calc_altitude(BME280 *dev, float pressure);
void bme280_set_sea_level(BME280 *dev, float pressure);

int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
