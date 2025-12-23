# BME280 Kalman Filter

Kalman filter implementation for altitude estimation using BME280 barometric pressure sensor.

## Features

- BME280 driver with temperature, pressure, and humidity compensation
- Linear Kalman filter for altitude smoothing
- Velocity estimation from pressure changes
- Portable C code for embedded systems

## Structure

```
bme280_kalman/
├── src/
│   ├── main.c
│   ├── kalman.c
│   ├── kalman.h
│   ├── bme280.c
│   └── bme280.h
├── CMakeLists.txt
├── README.md
└── USERGUIDE.md
```

## Quick Start

```bash
git clone https://github.com/yourusername/bme280_kalman.git
cd bme280_kalman
mkdir build
cd build
cmake ..
make
```

## Hardware

| BME280 Pin | MCU Pin |
|------------|---------|
| VCC        | 3.3V    |
| GND        | GND     |
| SCL        | I2C SCL |
| SDA        | I2C SDA |

## Configuration

Edit these values in `main.c`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| DT        | 0.1     | Sample time in seconds |
| Q_POS     | 0.01    | Process noise for position |
| Q_VEL     | 0.1     | Process noise for velocity |
| R_MEAS    | 9.0     | Measurement noise |

## Integration

Implement these functions for your platform:

```c
int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
void delay_ms(int ms);
```

## Output Format

```
T:25.32 P:1013.25 H:45.00 ALT:0.50 FALT:0.48 V:0.02
```

- T: Temperature (°C)
- P: Pressure (hPa)
- H: Humidity (%)
- ALT: Raw altitude (m)
- FALT: Filtered altitude (m)
- V: Velocity (m/s)

## License

MIT License
