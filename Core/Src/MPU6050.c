#include "../Inc/MPU6050.h"


// set i2c timeout
const uint16_t i2c_timeout = 100;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // wake sensor up using power management register
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register AFS_SEL=0
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register FS_SEL=0
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_MPU(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_R_GYRO_XOUT, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_R_ACCEL_XOUT, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);


}

void Calculate_Accel_Values(MPU6050_t *Raw_Values){
    //Scale values
	Raw_Values-> Accel_X = Raw_Values-> Accel_X_RAW*((2*G)/32768); //Values for AFS_SEL = 0
	Raw_Values-> Accel_Y = Raw_Values-> Accel_Y_RAW*((2*G)/32768);
	Raw_Values-> Accel_Z = Raw_Values-> Accel_Z_RAW*((2*G)/32768);
};

void Calculate_Gyro_Values(MPU6050_t *Raw_Values){
    //Scale values 
	Raw_Values-> Gyro_X = Raw_Values-> Gyro_X_RAW/131.0; //Values for FS_SEL = 0
	Raw_Values-> Gyro_Y = Raw_Values-> Gyro_Y_RAW/131.0;
	Raw_Values-> Gyro_Z = Raw_Values-> Gyro_Z_RAW/131.0;
};


void Get_Accel_Angles(MPU6050_t *Raw_Values){
    
    //     explanation in /Datasheets/MPU-6050-math.pdf
    
    // Calculate Pitch using RPY
	float pitch_tmp =  RAD_TO_DEG * asin(Raw_Values->Accel_X / G);
	Raw_Values->Accel_Pitch = Raw_Values->Accel_Pitch * (1 - ACCEL_LPF_ALPHA) + pitch_tmp * ACCEL_LPF_ALPHA;

    // Check for potential division by zero
    if (Raw_Values->Accel_Z != 0) {
    	float roll_tmp = RAD_TO_DEG * atan(Raw_Values->Accel_Y / Raw_Values->Accel_Z);
    	Raw_Values->Accel_Roll = Raw_Values->Accel_Roll * (1 - ACCEL_LPF_ALPHA) + roll_tmp * ACCEL_LPF_ALPHA;
    } else {
        // Handle division by zero error (if necessary)
    	Raw_Values->Accel_Roll = 0.0;  // Set a default value or handle it according to your application logic
    }
}



void Get_Gyro_Angles(MPU6050_t *Raw_Values, double Sample_Time){

    //     explanation in /Datasheets/MPU-6050-math.pdf


	float pitch_tmp = Raw_Values->Gyro_Pitch + Raw_Values->Gyro_Y*Sample_Time;
	Raw_Values->Gyro_Pitch = Raw_Values->Gyro_Pitch * (1 - GYRO_LPF_ALPHA) + pitch_tmp * GYRO_LPF_ALPHA;

	float roll_tmp = Raw_Values->Gyro_Roll + Raw_Values->Gyro_X*Sample_Time;
	Raw_Values->Gyro_Roll = Raw_Values->Gyro_Roll * (1 - GYRO_LPF_ALPHA) + roll_tmp * GYRO_LPF_ALPHA;

};

void Comp_Filter_Results(MPU6050_t *Results){
    
    //     explanation in /Datasheets/MPU-6050-math.pdf

	float pitch_tmp = Results->Gyro_Pitch * COMPLEMENTARY_ALPHA + (1-COMPLEMENTARY_ALPHA) * Results->Accel_Pitch;
	Results->Accel_Pitch = pitch_tmp;
	Results->Gyro_Pitch = pitch_tmp;

	float roll_tmp = Results->Gyro_Roll * COMPLEMENTARY_ALPHA + (1-COMPLEMENTARY_ALPHA) * Results->Accel_Roll;
	Results->Accel_Roll = roll_tmp;
	Results->Gyro_Roll = roll_tmp;

}


