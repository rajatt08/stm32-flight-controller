# STM32F407 Quadcopter Flight Controller

A bare-metal flight controller developed on STM32F407 using register-level programming.

## Project Overview

This project implements the core components of a quadcopter flight controller without using HAL libraries.

Features implemented:

- Register-level STM32F407 programming
- MPU6050 IMU interfacing using I2C
- Accelerometer and Gyroscope data acquisition
- Gyroscope calibration
- Complementary Filter for Roll and Pitch estimation
- PID stabilization for Roll, Pitch and Yaw
- PWM motor control using TIM4
- Four-motor mixing algorithm for quadcopter stabilization



## Hardware Used

- STM32F407 Discovery Board
- MPU6050 IMU
- ESCs (Electronic Speed Controllers)
- Brushless DC Motors
- LiPo Battery (planned)
- FlySky iBUS Receiver (planned)



## Software Architecture

### Sensor Layer
- I2C communication
- MPU6050 register access
- Accelerometer data reading
- Gyroscope data reading

### Estimation Layer
- Gyroscope offset calibration
- Complementary Filter
- Roll estimation
- Pitch estimation

### Control Layer
- Roll PID
- Pitch PID
- Yaw PID

### Output Layer
- PWM generation using TIM4
- Motor mixing
- ESC signal generation



## Current Flight Controller Logic

1. Read MPU6050 sensor data
2. Convert raw values into physical units
3. Estimate Roll and Pitch using Complementary Filter
4. Calculate PID corrections
5. Mix corrections into four motor outputs
6. Generate PWM signals for ESCs



## Implemented Modules

### GPIO Register Programming
- Direct register manipulation
- Alternate function configuration

### UART Communication
- USART2 transmitter
- Serial debugging

### I2C Communication
- Register-level I2C driver
- MPU6050 communication

### PWM Generation
- TIM4 based PWM
- ESC compatible pulse generation

### Sensor Fusion
- Complementary Filter

### Stabilization
- PID Controller



## Future Improvements

- iBUS Receiver Integration
- Real-time RC Control
- PID Tuning Interface
- Battery Voltage Monitoring
- Altitude Hold
- GPS Navigation
- Telemetry System


## Repository Structure

STM32-Flight-Controller/
│
├── Flight_Controller/
│   └── main.c
│
├── UART_Test/
│   └── main.c
│
├── I2C_Test/
│   └── main.c
│
├── Register_LED_Test/
│   └── main.c
│
└── README.md




## Learning Outcomes

Through this project I learned:

- Embedded C Programming
- STM32 Register Programming
- UART Communication
- I2C Protocol
- PWM Generation
- MPU6050 Interfacing
- Sensor Fusion
- PID Control
- Quadcopter Flight Control Fundamentals


## Author

Rajat Tripathi

Electronics Engineering (IoT)

Built as a self-learning project to understand the complete software stack of a quadcopter flight controller from scratch.
