# User Guide

## Overview

This library provides altitude estimation using a BME280 sensor and Kalman filter. The filter combines barometric pressure readings with a motion model to produce smooth altitude estimates and velocity calculations.

## Getting Started

### Prerequisites

- GCC compiler (or ARM toolchain for embedded)
- BME280 sensor connected via I2C
- I2C driver for your platform

### Building

Desktop simulation:
```bash
mkdir build
cd build
cmake ..
make
./bme280_kalman
```

Cross-compilation example:
```bash
mkdir build && cd build
cmake -DCMAKE_C_COMPILER=arm-none-eabi-gcc ..
make
```

## Platform Porting

### I2C Functions

You must implement two I2C functions:

```c
int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
```
Writes a single byte to a register.
- addr: I2C device address (0x76 for BME280)
- reg: Register address
- val: Value to write
- Returns: 0 on success, -1 on error

```c
int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
```
Reads multiple bytes from registers.
- addr: I2C device address
- reg: Starting register address
- buf: Buffer to store data
- len: Number of bytes to read
- Returns: 0 on success, -1 on error

### Delay Function

```c
void delay_ms(int ms)
```
Blocking delay in milliseconds.

### Example: STM32 HAL

```c
int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, 1, &val, 1, 100) == HAL_OK ? 0 : -1;
}

int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, 1, buf, len, 100) == HAL_OK ? 0 : -1;
}

void delay_ms(int ms) {
    HAL_Delay(ms);
}
```

### Example: Arduino

```c
#include <Wire.h>

int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0 ? 0 : -1;
}

int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, len);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return 0;
}

void delay_ms(int ms) {
    delay(ms);
}
```

## API Reference

### BME280 Functions

#### bme280_init
```c
int bme280_init(BME280 *dev)
```
Initializes the sensor and loads calibration data.
Returns 0 on success.

#### bme280_read_raw
```c
int bme280_read_raw(BME280 *dev, int32_t *temp, int32_t *press, int32_t *hum)
```
Reads raw ADC values from the sensor.

#### bme280_compensate_temp
```c
float bme280_compensate_temp(BME280 *dev, int32_t raw_temp)
```
Converts raw temperature to Celsius.
Must be called before pressure/humidity compensation.

#### bme280_compensate_press
```c
float bme280_compensate_press(BME280 *dev, int32_t raw_press)
```
Converts raw pressure to Pascals.

#### bme280_compensate_hum
```c
float bme280_compensate_hum(BME280 *dev, int32_t raw_hum)
```
Converts raw humidity to percentage.

#### bme280_calc_altitude
```c
float bme280_calc_altitude(BME280 *dev, float pressure)
```
Calculates altitude from pressure using barometric formula.

#### bme280_set_sea_level
```c
void bme280_set_sea_level(BME280 *dev, float pressure)
```
Sets reference sea level pressure for altitude calculation.

### Kalman Filter Functions

#### kalman_init
```c
void kalman_init(KalmanFilter *kf, float dt, float q_pos, float q_vel, float r_meas)
```
Initializes the filter.
- dt: Time step in seconds
- q_pos: Process noise for position
- q_vel: Process noise for velocity
- r_meas: Measurement noise

#### kalman_predict
```c
void kalman_predict(KalmanFilter *kf, float acceleration)
```
Prediction step. Pass 0 if no accelerometer available.

#### kalman_update
```c
void kalman_update(KalmanFilter *kf, float measurement)
```
Update step with altitude measurement.

#### kalman_get_altitude
```c
float kalman_get_altitude(KalmanFilter *kf)
```
Returns filtered altitude estimate.

#### kalman_get_velocity
```c
float kalman_get_velocity(KalmanFilter *kf)
```
Returns estimated vertical velocity.

## Tuning

### Process Noise (Q)

Q_POS and Q_VEL control how much the filter trusts the motion model.
- Higher values: Filter responds faster, more noise in output
- Lower values: Smoother output, slower response

### Measurement Noise (R)

R_MEAS controls how much the filter trusts the sensor.
- Higher values: More smoothing, ignores sensor noise
- Lower values: Follows sensor closely

### Recommended Starting Points

| Application | Q_POS | Q_VEL | R_MEAS |
|-------------|-------|-------|--------|
| Static      | 0.001 | 0.01  | 9.0    |
| Walking     | 0.01  | 0.1   | 9.0    |
| Rocket      | 0.1   | 1.0   | 9.0    |
| Drone       | 0.05  | 0.5   | 4.0    |

## Troubleshooting

### Sensor Not Found
- Check I2C connections
- Verify address (0x76 or 0x77)
- Add pull-up resistors (4.7k)

### Altitude Drift
- Allow 1-2 minutes warm-up
- Recalibrate sea level periodically
- Temperature changes affect pressure

### Slow Response
- Decrease Q values
- Reduce sample time (DT)

### Noisy Output
- Increase R_MEAS
- Decrease Q values
- Check for EMI near sensor
