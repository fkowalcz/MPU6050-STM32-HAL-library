#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "MPU6050-REGISTER-MAP.h"

#define G  9.81
#define RAD_TO_DEG 57.29

#define AFS_SEL_0 16384
#define AFS_SEL_1 8192
#define AFS_SEL_2 4096
#define AFS_SEL_3 2048



#define ACCEL_LPF_ALPHA 0.100000000f
#define GYRO_LPF_ALPHA 0.950000000f
#define COMPLEMENTARY_ALPHA 0.09090909f


typedef struct
{
	int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;

    float Gyro_Pitch;
    float Gyro_Roll;



	int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    float Accel_X;
    float Accel_Y;
    float Accel_Z;

    float Accel_Pitch;
    float Accel_Roll;



} MPU6050_t;
// Sensor board initialization
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

// Reading raw sensor values
void Calculate_Accel_Values(MPU6050_t *Raw_Values);

void Calculate_Gyro_Values(MPU6050_t *Raw_Values);

void MPU6050_Read_MPU(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

// Calculating the spatial position of the sensor 
void Get_Accel_Angles(MPU6050_t *Raw_Values);

void Get_Gyro_Angles(MPU6050_t *Raw_Values, double Sample_Time);

// Complementary filter implementation for accelerometer and gyroscope fussion
void Comp_Filter_Results(MPU6050_t *Results);


