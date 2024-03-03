#ifndef INC_MPU6050_REGISTER_MAP_H_
#define INC_MPU6050_REGISTER_MAP_H_

#define SELF_TEST_X 							(0x0D)
#define SELF_TEST_Y 							(0x0E)
#define SELF_TEST_Z 							(0x0F)
#define SELF_TEST_A 							(0x10)
#define SMPLRT_DIV 								(0x19)
#define CONFIG 									(0x1A)
#define GYRO_CONFIG 							(0x1B)
#define ACCEL_CONFIG 							(0x1C)
#define MOT_THR 								(0x1F)
#define FIFO_EN 								(0x23)
#define I2C_MST_CTRL 							(0x24)
#define I2C_SLV0_ADDR 							(0x25)
#define I2C_SLV0_REG 							(0x26)
#define I2C_SLV0_CTRL 							(0x27)
#define I2C_SLV1_ADDR 							(0x28)
#define I2C_SLV1_REG 							(0x29)
#define I2C_SLV1_CTRL 							(0x2A)
#define I2C_SLV2_ADDR 							(0x2B)
#define I2C_SLV2_REG 							(0x2C)
#define I2C_SLV2_CTRL 							(0x2D)
#define I2C_SLV3_ADDR 							(0x2E)
#define I2C_SLV3_REG 							(0x2E)
#define I2C_SLV3_CTRL 							(0x30)
#define I2C_SLV4_ADDR 							(0x31)
#define I2C_SLV4_REG 							(0x32)
#define I2C_SLV4_DO 							(0x33)
#define I2C_SLV4_CTRL 							(0x34)
#define I2C_SLV4_DI 							(0x35)
#define I2C_MST_STATUS 							(0x36)
#define INT_PIN_CFG 							(0x37)
#define INT_ENABLE 								(0x37)

#define ACCEL_XOUT_H_R_ACCEL_XOUT 				(0x3B) // [15:8]
#define ACCEL_XOUT_L_R_ACCEL_XOUT 				(0x3C) // [7:0]
#define ACCEL_YOUT_H_R_ACCEL_YOUT 				(0x3D) // [15:8]
#define ACCEL_YOUT_L_R_ACCEL_YOUT 				(0x3E) // [7:0]
#define ACCEL_ZOUT_H_R_ACCEL_ZOUT 				(0x3F) // [15:8]
#define ACCEL_ZOUT_L_R_ACCEL_ZOUT 				(0x40) // [7:0]
#define TEMP_OUT_H_R_TEMP_OUT 					(0x41) // [15:8]
#define TEMP_OUT_L_R_TEMP_OUT 					(0x42) // [7:0]
#define GYRO_XOUT_H_R_GYRO_XOUT 				(0x43) // [15:8]
#define GYRO_XOUT_L_R_GYRO_XOUT 				(0x44) // [7:0]
#define GYRO_YOUT_H_R_GYRO_YOUT 				(0x45) // [15:8]
#define GYRO_YOUT_L_R_GYRO_YOUT 				(0x46) // [7:0]
#define GYRO_ZOUT_H_R_GYRO_ZOUT 				(0x47) // [15:8]
#define GYRO_ZOUT_L_R_GYRO_ZOUT 				(0x48) // [7:0]
#define EXT_SENS_DATA_00_R_EXT_SENS_DATA_00 	(0x49) // [7:0]
#define EXT_SENS_DATA_01_R_EXT_SENS_DATA_01 	(0x4A) // [7:0]
#define EXT_SENS_DATA_02_R_EXT_SENS_DATA_02 	(0x4B) // [7:0]
#define EXT_SENS_DATA_03_R_EXT_SENS_DATA_03 	(0x4C) // [7:0]
#define EXT_SENS_DATA_04_R_EXT_SENS_DATA_04 	(0x4D) // [7:0]
#define EXT_SENS_DATA_05_R_EXT_SENS_DATA_05 	(0x4E) // [7:0]
#define EXT_SENS_DATA_06_R_EXT_SENS_DATA_06 	(0x4F) // [7:0]
#define EXT_SENS_DATA_07_R_EXT_SENS_DATA_07 	(0x50) // [7:0]
#define EXT_SENS_DATA_08_R_EXT_SENS_DATA_08 	(0x51) // [7:0]
#define EXT_SENS_DATA_09_R_EXT_SENS_DATA_09 	(0x52) // [7:0]
#define EXT_SENS_DATA_10_R_EXT_SENS_DATA_10 	(0x53) // [7:0]
#define EXT_SENS_DATA_11_R_EXT_SENS_DATA_11 	(0x54) // [7:0]
#define EXT_SENS_DATA_12_R_EXT_SENS_DATA_12 	(0x55) // [7:0]
#define EXT_SENS_DATA_13_R_EXT_SENS_DATA_13 	(0x56) // [7:0]
#define EXT_SENS_DATA_14_R_EXT_SENS_DATA_14 	(0x57) // [7:0]
#define EXT_SENS_DATA_15_R_EXT_SENS_DATA_15 	(0x58) // [7:0]
#define EXT_SENS_DATA_16_R_EXT_SENS_DATA_16 	(0x59) // [7:0]
#define EXT_SENS_DATA_17_R_EXT_SENS_DATA_17 	(0x5A) // [7:0]
#define EXT_SENS_DATA_18_R_EXT_SENS_DATA_18 	(0x5B) // [7:0]
#define EXT_SENS_DATA_19_R_EXT_SENS_DATA_19 	(0x5C) // [7:0]
#define EXT_SENS_DATA_20_R_EXT_SENS_DATA_20 	(0x5D) // [7:0]
#define EXT_SENS_DATA_21_R_EXT_SENS_DATA_21 	(0x5E) // [7:0]
#define EXT_SENS_DATA_22_R_EXT_SENS_DATA_22 	(0x5F) // [7:0]
#define EXT_SENS_DATA_23_R_EXT_SENS_DATA_23 	(0x60) // [7:0]
#define I2C_SLV0_DO_RW_I2C_SLV0_DO 				(0x63) // [7:0]
#define I2C_SLV1_DO_RW_I2C_SLV1_DO 				(0x64) // [7:0]

#define I2C_SLV2_DO								(0x65)
#define I2C_SLV3_DO								(0x66)
#define I2C_MST_DELAY_CT_LR						(0x67)
#define SIGNAL_PATH_RES_ET						(0x68)
#define MOT_DETECT_CTRL							(0x69)
#define USER_CTRL								(0x6A)
#define PWR_MGMT_1								(0x6B)
#define PWR_MGMT_2								(0x6C)
#define FIFO_COUNTH								(0x72)
#define FIFO_COUNTL								(0x73)
#define FIFO_R_W								(0x74)
#define WHO_AM_I								(0x75)

#define MPU6050_ADDR 0xD0


#endif /* INC_MPU6050_REGISTER_MAP_H_ */
