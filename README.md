# Kalman Filter for Rocket Altitude Estimation

A collection of Kalman filter implementations for altitude estimation using barometric pressure sensors and accelerometer data. Designed for model rocketry and high-power rocketry applications.

## Overview

This project provides multiple implementations of a linear Kalman filter for fusing barometer and accelerometer data to estimate altitude and velocity during rocket flight. The filter smooths noisy sensor readings to provide accurate real-time altitude tracking.

## Features

- **Sensor Fusion**: Combines barometer altitude with accelerometer data
- **Noise Reduction**: Filters out sensor noise while preserving actual flight dynamics
- **Velocity Estimation**: Derives velocity from filtered altitude data
- **Multiple Implementations**: Python simulations and embedded C code
- **Real-time Processing**: Suitable for onboard flight computers

## Repository Structure

```
KALMAN_Filter/
├── kalman.py                  # Basic Kalman filter simulation
├── kalmanDummySim.py          # Rocket flight trajectory simulator
├── micro_py_Implementation.py # Real-time serial interface for microcontrollers
├── rocket_dummy_flight.csv    # Sample flight data
├── README.md                  # This file
└── bme280_kalman/             # Embedded C implementation
    ├── src/
    │   ├── main.c             # Main application
    │   ├── kalman.c           # Kalman filter implementation
    │   ├── kalman.h           # Kalman filter header
    │   ├── bme280.c           # BME280 sensor driver
    │   └── bme280.h           # BME280 header
    ├── CMakeLists.txt         # Build configuration
    ├── README.md              # Embedded implementation docs
    └── USERGUIDE.md           # Detailed usage guide
```

## Quick Start

### Python Simulation

1. Install dependencies:
   ```bash
   pip install numpy matplotlib pandas
   ```

2. Generate dummy flight data:
   ```bash
   python kalmanDummySim.py
   ```

3. Run the Kalman filter simulation:
   ```bash
   python kalman.py
   ```

### Real-time Microcontroller Interface

1. Connect your microcontroller (e.g., ESP32) via serial
2. Configure the serial port in `micro_py_Implementation.py`:
   ```python
   SERIAL_PORT = 'COM3'  # Adjust for your system
   BAUD_RATE = 115200
   ```
3. Run the real-time filter:
   ```bash
   python micro_py_Implementation.py
   ```

### Embedded C Implementation

See [bme280_kalman/README.md](bme280_kalman/README.md) for building and deploying to embedded systems.

## How It Works

### State Space Model

The Kalman filter uses a constant acceleration model:

**State Vector:**
```
x = [altitude, velocity]ᵀ
```

**State Transition:**
```
A = | 1   dt |
    | 0   1  |
```

**Control Input (Acceleration):**
```
B = | 0.5·dt² |
    | dt      |
```

**Measurement Model:**
```
H = | 1   0 |  (barometer measures altitude only)
```

### Filter Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `dt`      | Sample time | 0.1 s |
| `Q`       | Process noise covariance | Tune based on model trust |
| `R`       | Measurement noise | ~3-9 m² (barometer variance) |

### Tuning Guidelines

- **Increase Q**: Filter trusts measurements more, faster response but more noise
- **Increase R**: Filter trusts model more, smoother output but slower response
- **Barometer noise**: Typically 1-5 meters standard deviation
- **IMU noise**: Typically 0.1-0.5 m/s² standard deviation

## Flight Phases

The filter handles all typical rocket flight phases:

1. **Launch/Thrust**: High acceleration, rapid altitude change
2. **Coast**: Deceleration due to gravity and drag
3. **Apogee**: Near-zero velocity detection
4. **Descent**: Negative velocity, chute deployment detection

## Hardware Compatibility

### Supported Sensors
- **BME280**: Barometric pressure, temperature, humidity
- **BMP280**: Barometric pressure, temperature
- **MPU6050/MPU9250**: Accelerometer/Gyroscope
- Any I2C/SPI barometer + accelerometer combination

### Tested Microcontrollers
- ESP32
- STM32
- Arduino (with modifications)
- Raspberry Pi Pico

## Sample Output

```
Alt: 150.23 m | Vel: 45.67 m/s
Alt: 155.89 m | Vel: 44.12 m/s
Alt: 160.34 m | Vel: 42.58 m/s
...
```

## Visualization

The Python implementations include matplotlib visualization showing:
- True altitude (if simulated)
- Raw barometer readings
- Kalman filtered estimate

![Kalman Filter Output](https://via.placeholder.com/600x300?text=Kalman+Filter+Altitude+Estimation)

## Contributing

Contributions are welcome! Please feel free to submit pull requests for:
- Additional sensor drivers
- Platform-specific implementations
- Performance optimizations
- Documentation improvements

## License

MIT License - See individual files for details.

## References

- [Kalman Filter Wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)
- [BME280 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
- [Understanding Kalman Filters (MATLAB)](https://www.mathworks.com/videos/understanding-kalman-filters-part-1-why-use-kalman-filters--1485813028675.html)

## Author

[oslowtech](https://github.com/oslowtech)
