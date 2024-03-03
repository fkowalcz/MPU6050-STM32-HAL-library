# MPU-6050 Library for STM HAL

This library provides functionality to interface with the MPU-6050 sensor using STM HAL. It facilitates the initialization of the sensor, reading gyroscope and accelerometer measurements, and calculating orientation using these two sensors. The mathematical explanation behind translating measurements to orientation in degrees is detailed in the file [MPU6050-math.pdf](/Documents/MPU6050-math.pdf).

## Installation

To use this library, simply add the provided files to your STM32 project and include them in your build configuration. If you are using different ST microcontroller you might need to change the line 
``` c
#include "stm32l4xx_hal.h"
```
in [MPU6050.h](/Core/Inc/MPU6050.h)  to HAL version matching your microcontroller.


## Usage

An example of usage is presented in `main.c`. Here's how you can use the library in your STM32 project:

1. Initialize the MPU-6050 sensor using the `MPU6050_Init` function.
2. Read gyroscope and accelerometer measurements using the `MPU6050_Read_MPU` function.
3. Calculate measurements after scaling using `Calculate_Accel_Values` and `Calculate_Gyro_Values` funtions, respectively.
4. Calculate orientation using using `Get_Accel_Angles` and `Get_Gyro_Angles` funtions, respectively. (Gyroscope measure requires fixed sampling time. Example presented in 'main.c' uses timer interrupts)
5. Use `Comp_Filter_Results` function to eliminate sensor drift. (Complementary filter isn't tunned yet)

Example presented in `main.c` contains sending measured data to uart which can be used to save series of measurements necessary for tuning LPF, HPF and complementary filter.
	

