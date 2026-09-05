
/*
 * STM32F407 Flight Controller
 *
 * Features:
 * - Register Level Programming
 * - MPU6050 IMU Interface (I2C)
 * - Complementary Filter
 * - Roll/Pitch/Yaw Estimation
 * - PID Stabilization
 * - PWM Motor Control (TIM4)
 *
 * Author: Rajat Tripathi
 */


#include "stm32f407xx.h"
#include <stdint.h>
#include <math.h>

void I2C1_Init(void){


	RCC->AHB1ENR |= (1 << 1);   // GPIOB clock enable

	RCC->APB1ENR |= (1 << 21);  // I2C1 clock enable

	GPIOB->MODER &= ~(3 << (6*2));   // clear first for PB6
	GPIOB->MODER |=  (2 << (6*2));   // set as alternate fn

	GPIOB->MODER &= ~(3 << (7*2));    // clear first for PB7
	GPIOB->MODER |=  (2 << (7*2));    // set as alternate fn

	GPIOB->AFR[0] &= ~(0xF << 24);   // pb6 from 24 to 27 clear
	GPIOB->AFR[0] |=  (4   << 24);   // now configure AF4 = I2C1_SCL

	GPIOB->AFR[0] &= ~(0xF << 28);   // pb7 from 28 to 31
	GPIOB->AFR[0] |=  (4   << 28);  // now configure AF4 = I2C1_SDA

	GPIOB->OTYPER |= (1<<6);   // PB6 = OPEN DRAIN
	GPIOB->OTYPER |= (1<<7);   // PB7 = OPEN DRAIN

	I2C1->CR2   = 42;
	I2C1->CCR   = 210;
	I2C1->TRISE = 43;

	I2C1->CR1 |= (1<<0);   // ENABLE THE PERIPHERAL OF I2C1


}

void I2C1_Start(void)
{
    I2C1->CR1 |= (1<<8);      // Generate START

    while(!(I2C1->SR1 & (1<<0)));
}

void I2C1_SendAddress(uint8_t address)
{
    volatile uint32_t temp;

    I2C1->DR = address;

    while(!(I2C1->SR1 & (1<<1)));

    temp = I2C1->SR1;
    temp = I2C1->SR2;
}

void I2C1_Write(uint8_t data)
{
    I2C1->DR = data;

    while(!(I2C1->SR1 & (1<<7)));   // Wait for TXE
}

uint8_t I2C1_Read(void)
{
    while(!(I2C1->SR1 & (1<<6)));   // Wait for RXNE

    return I2C1->DR;
}

void I2C1_Stop(void)
{
    I2C1->CR1 |= (1<<9);
}

/* We need Repeated START to switch from Write operation
   (register selection) to Read operation (getting the data)
   without ending the transaction */


uint8_t MPU6050_ReadRegister(uint8_t reg)
{
    uint8_t data;

    I2C1_Start();

    I2C1_SendAddress(0xD0);   // Write mode

    I2C1_Write(reg);

    I2C1_Start();             // Repeated START

    I2C1_SendAddress(0xD1);   // Read mode

    data = I2C1_Read();

    I2C1_Stop();

    return data;
}

void MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
    I2C1_Start();

    I2C1_SendAddress(0xD0);

    I2C1_Write(reg);

    I2C1_Write(value);

    I2C1_Stop();
}

int16_t MPU6050_Read16Bit(uint8_t reg)
{
    uint8_t high;
    uint8_t low;

    high = MPU6050_ReadRegister(reg);
    low  = MPU6050_ReadRegister(reg + 1);

    return (high << 8) | low;
}

void TIM4_PWM_Init(void)
{
    RCC->APB1ENR |= (1<<2);

    RCC->AHB1ENR |= (1<<3);   // GPIOD clock enable

    GPIOD->MODER &= ~(3 << (12*2));
    GPIOD->MODER |=  (2 << (12*2));

    GPIOD->AFR[1] &= ~(0xF << 16);
    GPIOD->AFR[1] |=  (2 << 16);

    TIM4->PSC = 84 - 1;
    TIM4->ARR = 20000 - 1;

    TIM4->CCMR1 &= ~(7 << 4);
    TIM4->CCMR1 |=  (6 << 4);

    TIM4->CCER |= (1 << 0);

    TIM4->CCR1 = 1000;

    TIM4->CR1 |= (1 << 0);

    // pd13
    GPIOD->MODER &= ~(3 << (13*2));
    GPIOD->MODER |=  (2 << (13*2));

    GPIOD->AFR[1] &= ~(0xF << 20);
    GPIOD->AFR[1] |=  (2 << 20);

    //pd14
    GPIOD->MODER &= ~(3 << (14*2));
    GPIOD->MODER |=  (2 << (14*2));

    GPIOD->AFR[1] &= ~(0xF << 24);
    GPIOD->AFR[1] |=  (2 << 24);

    //pd15
    GPIOD->MODER &= ~(3 << (15*2));
    GPIOD->MODER |=  (2 << (15*2));

    GPIOD->AFR[1] &= ~(0xF << 28);
    GPIOD->AFR[1] |=  (2 << 28);

    //ch2 pwm mode
    TIM4->CCMR1 &= ~(7 << 12);
    TIM4->CCMR1 |=  (6 << 12);

    //ch3 pwm mode
    TIM4->CCMR2 &= ~(7 << 4);
    TIM4->CCMR2 |=  (6 << 4);

    //ch4 pwm mode
    TIM4->CCMR2 &= ~(7 << 12);
    TIM4->CCMR2 |=  (6 << 12);

    // enable ch2 ch3 ch4
    TIM4->CCER |= (1 << 4);
    TIM4->CCER |= (1 << 8);
    TIM4->CCER |= (1 << 12);

    // initial pwm values
    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
    TIM4->CCR3 = 1000;
    TIM4->CCR4 = 1000;

    TIM4->CCMR1 |= (1 << 3);
    TIM4->CCMR1 |= (1 << 11);

    TIM4->CCMR2 |= (1 << 3);
    TIM4->CCMR2 |= (1 << 11);

    TIM4->CR1 |= (1 << 7);

    TIM4->EGR |= (1 << 0);

    // initial pwm values
    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
    TIM4->CCR3 = 1000;
    TIM4->CCR4 = 1000;

    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
    TIM4->CCR3 = 1000;
    TIM4->CCR4 = 1000;

    for(volatile int i=0; i<5000000; i++);
}

TIM4->CCR1 = 1000;
TIM4->CCR2 = 1000;
TIM4->CCR3 = 1000;
TIM4->CCR4 = 1000;

for(volatile int i=0;i<20000000;i++);





int main(void)
{
    uint8_t data;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;



    float roll = 0;
    float pitch = 0;

    float roll_acc;
    float pitch_acc;

    float dt = 0.01f;

    float kp = 2.0f;
    float ki = 0.1f;
    float kd = 0.5f;

    float roll_setpoint = 0.0f;

    float roll_error;
    float roll_integral = 0;
    float roll_derivative;
    float previous_error = 0;

     int throttle = 1500;


    int motor1;
    int motor2;
    int motor3;
    int motor4;

    float roll_output;

    float pitch_setpoint = 0;

    float pitch_error;
    float pitch_integral = 0;
    float pitch_derivative;
    float previous_pitch_error = 0;

    float pitch_output;

    float yaw = 0;

    float yaw_setpoint = 0;

    float yaw_error;

    float yaw_integral = 0;

    float yaw_derivative;

    float previous_yaw_error = 0;

    float yaw_output;

    float gyro_x_offset = 0;
    float gyro_y_offset = 0;
    float gyro_z_offset = 0;

    I2C1_Init();

    TIM4_PWM_Init();

    MPU6050_WriteRegister(0x6B, 0x00);   // Wake MPU6050

    data = MPU6050_ReadRegister(0x75);

    if(data != 0x68)
    {
        while(1);
    }

    for(int i=0;i<1000;i++)
    {
        gyro_x_offset += MPU6050_Read16Bit(0x43);
        gyro_y_offset += MPU6050_Read16Bit(0x45);
        gyro_z_offset += MPU6050_Read16Bit(0x47);

        for(volatile int d=0; d<5000; d++);
    }

    gyro_x_offset /= 1000.0f;
    gyro_y_offset /= 1000.0f;
    gyro_z_offset /= 1000.0f;





    while(1)
    {

    	accel_x = MPU6050_Read16Bit(0x3B);
    	accel_y = MPU6050_Read16Bit(0x3D);
    	accel_z = MPU6050_Read16Bit(0x3F);

    	gyro_x  = MPU6050_Read16Bit(0x43);
    	gyro_y  = MPU6050_Read16Bit(0x45);
    	gyro_z  = MPU6050_Read16Bit(0x47);

    	accel_x_g = accel_x / 16384.0f;
    	accel_y_g = accel_y / 16384.0f;
    	accel_z_g = accel_z / 16384.0f;

    	gyro_x_dps = (gyro_x - gyro_x_offset)/131.0f;
    	gyro_y_dps = (gyro_y - gyro_y_offset)/131.0f;
    	gyro_z_dps = (gyro_z - gyro_z_offset)/131.0f;

    	roll_acc = atan2(accel_y_g, accel_z_g) * 57.2958f;

    	pitch_acc = atan2(
    	                -accel_x_g,
    	                sqrt(accel_y_g*accel_y_g +
    	                     accel_z_g*accel_z_g)
    	            ) * 57.2958f;

    	roll =
    	    0.98f * (roll + gyro_x_dps * dt)
    	    + 0.02f * roll_acc;

    	pitch =
    	    0.98f * (pitch + gyro_y_dps * dt)
    	    + 0.02f * pitch_acc;

    	roll_error = roll_setpoint - roll;

    	roll_integral += roll_error * dt;

    	if(roll_integral > 100)
    	    roll_integral = 100;

    	if(roll_integral < -100)
    	    roll_integral = -100;

    	roll_derivative =
    	    (roll_error - previous_error) / dt;

    	roll_output =
    	    (kp * roll_error)
    	    +
    	    (ki * roll_integral)
    	    +
    	    (kd * roll_derivative);

    	previous_error = roll_error;

    	pitch_error = pitch_setpoint - pitch;

    	pitch_integral += pitch_error * dt;

    	if(pitch_integral > 100)
    	    pitch_integral = 100;

    	if(pitch_integral < -100)
    	    pitch_integral = -100;

    	pitch_derivative =
    	    (pitch_error - previous_pitch_error) / dt;

    	pitch_output =
    	    (kp * pitch_error)
    	    +
    	    (ki * pitch_integral)
    	    +
    	    (kd * pitch_derivative);

    	previous_pitch_error = pitch_error;

    	yaw += gyro_z_dps * dt;

    	yaw_error = yaw_setpoint - yaw;

    	yaw_integral += yaw_error * dt;

    	if(yaw_integral > 100)
    	    yaw_integral = 100;

    	if(yaw_integral < -100)
    	    yaw_integral = -100;

    	yaw_derivative =
    	    (yaw_error - previous_yaw_error) / dt;

    	yaw_output =
    	    (kp * yaw_error)
    	    +
    	    (ki * yaw_integral)
    	    +
    	    (kd * yaw_derivative);

    	previous_yaw_error = yaw_error;

    	motor1 = throttle + pitch_output + roll_output + yaw_output;

    	motor2 = throttle + pitch_output - roll_output - yaw_output;

    	motor3 = throttle - pitch_output - roll_output + yaw_output;

    	motor4 = throttle - pitch_output + roll_output - yaw_output;

    	if(motor1 > 2000) motor1 = 2000;
    	if(motor1 < 1000) motor1 = 1000;

    	if(motor2 > 2000) motor2 = 2000;
    	if(motor2 < 1000) motor2 = 1000;

    	if(motor3 > 2000) motor3 = 2000;
    	if(motor3 < 1000) motor3 = 1000;

    	if(motor4 > 2000) motor4 = 2000;
    	if(motor4 < 1000) motor4 = 1000;

    	//pwm registers update
    	TIM4->CCR1 = motor1;
    	TIM4->CCR2 = motor2;
    	TIM4->CCR3 = motor3;
    	TIM4->CCR4 = motor4;

    	for(volatile int i=0;i<50000;i++);

    }
}
