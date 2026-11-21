# STM32 Quadcopter Flight Controller & RF Remote

Firmware for a DIY quadcopter on STM32: IMU-based attitude estimation (complementary filter), per-axis PID stabilization and a 2.4 GHz NRF24L01 control link from a handheld remote.

> This repository currently contains the **drone-side firmware**. Remote-controller firmware will be added later.

## System overview

```
  Remote (STM32)          2.4 GHz           Drone (STM32)
 +--------------+    NRF24L01 (SPI)    +-----------------------------------+
 | Joysticks    | -------------------> | RX: throttle / roll / pitch / yaw |
 +--------------+                      | MPU6050 (I2C) -> angle estimate   |
                                       | PID roll / pitch / yaw            |
                                       | -> PWM to 4 x ESC                 |
                                       +-----------------------------------+
```

| Component | Role | Interface |
|---|---|---|
| STM32 MCU | Control loop, motor PWM | — |
| MPU6050 | Accelerometer + gyroscope | I2C |
| NRF24L01 | Wireless link remote ↔ drone | SPI |
| Character LCD + I2C backpack | Status / calibration display | I2C |

## Control loop

1. **Sense** — read MPU6050, fuse accelerometer and gyro (complementary filter) into roll/pitch angles.
2. **Receive** — decode throttle, roll, pitch and yaw set-points from the NRF24L01 payload.
3. **Control** — PID per axis computes the correction from set-point vs. measured angle.
4. **Actuate** — mix outputs into PWM duty cycles for the four ESCs.

## Repository structure

```
firmware/
├── Inc/
│   ├── main.h
│   ├── pid.h
│   ├── mpu6050.h
│   ├── NRF24.h, NRF24_conf.h, NRF24_reg_addresses.h
│   └── i2c_lcd.h
└── Src/
    ├── main.c        # Init, control loop
    ├── pid.c         # PID controller
    ├── mpu6050.c     # IMU driver + angle estimation
    ├── NRF24.c       # NRF24L01 driver
    └── i2c_lcd.c     # LCD driver
```

> Add `firmware/Inc` to the include path when importing into STM32CubeIDE.
