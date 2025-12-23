#include "bme280.h"
#include <math.h>

static void load_calibration(BME280 *dev, uint8_t *buf1, uint8_t *buf2) {
    dev->calib.dig_T1 = (uint16_t)(buf1[1] << 8 | buf1[0]);
    dev->calib.dig_T2 = (int16_t)(buf1[3] << 8 | buf1[2]);
    dev->calib.dig_T3 = (int16_t)(buf1[5] << 8 | buf1[4]);
    
    dev->calib.dig_P1 = (uint16_t)(buf1[7] << 8 | buf1[6]);
    dev->calib.dig_P2 = (int16_t)(buf1[9] << 8 | buf1[8]);
    dev->calib.dig_P3 = (int16_t)(buf1[11] << 8 | buf1[10]);
    dev->calib.dig_P4 = (int16_t)(buf1[13] << 8 | buf1[12]);
    dev->calib.dig_P5 = (int16_t)(buf1[15] << 8 | buf1[14]);
    dev->calib.dig_P6 = (int16_t)(buf1[17] << 8 | buf1[16]);
    dev->calib.dig_P7 = (int16_t)(buf1[19] << 8 | buf1[18]);
    dev->calib.dig_P8 = (int16_t)(buf1[21] << 8 | buf1[20]);
    dev->calib.dig_P9 = (int16_t)(buf1[23] << 8 | buf1[22]);
    
    dev->calib.dig_H1 = buf1[25];
    dev->calib.dig_H2 = (int16_t)(buf2[1] << 8 | buf2[0]);
    dev->calib.dig_H3 = buf2[2];
    dev->calib.dig_H4 = (int16_t)(buf2[3] << 4 | (buf2[4] & 0x0F));
    dev->calib.dig_H5 = (int16_t)(buf2[5] << 4 | (buf2[4] >> 4));
    dev->calib.dig_H6 = (int8_t)buf2[6];
}

int bme280_init(BME280 *dev) {
    uint8_t id;
    uint8_t calib1[26];
    uint8_t calib2[7];
    
    if (i2c_read_reg(BME280_ADDR, BME280_REG_ID, &id, 1) != 0) {
        return -1;
    }
    
    if (id != 0x60) {
        return -1;
    }
    
    i2c_write_reg(BME280_ADDR, BME280_REG_RESET, 0xB6);
    
    for (volatile int i = 0; i < 100000; i++);
    
    if (i2c_read_reg(BME280_ADDR, BME280_REG_CALIB_00, calib1, 26) != 0) {
        return -1;
    }
    
    if (i2c_read_reg(BME280_ADDR, BME280_REG_CALIB_26, calib2, 7) != 0) {
        return -1;
    }
    
    load_calibration(dev, calib1, calib2);
    
    i2c_write_reg(BME280_ADDR, BME280_REG_CTRL_HUM, 0x01);
    i2c_write_reg(BME280_ADDR, BME280_REG_CONFIG, 0xA0);
    i2c_write_reg(BME280_ADDR, BME280_REG_CTRL_MEAS, 0x27);
    
    dev->sea_level_pressure = 101325.0f;
    
    return 0;
}

int bme280_read_raw(BME280 *dev, int32_t *temp, int32_t *press, int32_t *hum) {
    uint8_t buf[8];
    
    if (i2c_read_reg(BME280_ADDR, BME280_REG_PRESS_MSB, buf, 8) != 0) {
        return -1;
    }
    
    *press = (int32_t)((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 4;
    *temp = (int32_t)((buf[3] << 16) | (buf[4] << 8) | buf[5]) >> 4;
    *hum = (int32_t)(buf[6] << 8 | buf[7]);
    
    (void)dev;
    
    return 0;
}

float bme280_compensate_temp(BME280 *dev, int32_t raw_temp) {
    int32_t var1, var2;
    
    var1 = ((((raw_temp >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * 
            ((int32_t)dev->calib.dig_T2)) >> 11;
    
    var2 = (((((raw_temp >> 4) - ((int32_t)dev->calib.dig_T1)) * 
              ((raw_temp >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) * 
            ((int32_t)dev->calib.dig_T3)) >> 14;
    
    dev->t_fine = var1 + var2;
    
    return (float)((dev->t_fine * 5 + 128) >> 8) / 100.0f;
}

float bme280_compensate_press(BME280 *dev, int32_t raw_press) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + 
           ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0;
    }
    
    p = 1048576 - raw_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);
    
    return (float)p / 256.0f;
}

float bme280_compensate_hum(BME280 *dev, int32_t raw_hum) {
    int32_t v_x1;
    
    v_x1 = dev->t_fine - 76800;
    
    v_x1 = (((((raw_hum << 14) - (((int32_t)dev->calib.dig_H4) << 20) - 
              (((int32_t)dev->calib.dig_H5) * v_x1)) + 16384) >> 15) * 
            (((((((v_x1 * ((int32_t)dev->calib.dig_H6)) >> 10) * 
                (((v_x1 * ((int32_t)dev->calib.dig_H3)) >> 11) + 32768)) >> 10) + 
              2097152) * ((int32_t)dev->calib.dig_H2) + 8192) >> 14));
    
    v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * 
                    ((int32_t)dev->calib.dig_H1)) >> 4);
    
    if (v_x1 < 0) v_x1 = 0;
    if (v_x1 > 419430400) v_x1 = 419430400;
    
    return (float)(v_x1 >> 12) / 1024.0f;
}

float bme280_calc_altitude(BME280 *dev, float pressure) {
    return 44330.0f * (1.0f - powf(pressure / dev->sea_level_pressure, 0.1903f));
}

void bme280_set_sea_level(BME280 *dev, float pressure) {
    dev->sea_level_pressure = pressure;
}
