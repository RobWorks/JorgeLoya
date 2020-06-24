/*
 * MPU9250.h
 *
 *  Created on: 29/11/2019
 *      Author: loya_
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "DataTypeDefinitions.h"
#include "fsl_i2c.h"

#define MPU9250_ADDR 0x68

typedef enum
{
  RANGE_16G          = 0b11,
  RANGE_8G           = 0b10,
  RANGE_4G           = 0b01,
  RANGE_2G           = 0b00
} accel_range;

typedef enum
{
  RANGE_GYRO_2000    = 0b11,
  RANGE_GYRO_1000    = 0b10,
  RANGE_GYRO_500     = 0b01,
  RANGE_GYRO_250     = 0b00
} gyro_range;

typedef enum
{
  SCALE_14_BITS      = 0,
  SCALE_16_BITS      = 1
} mag_scale;

typedef enum
{
  MAG_8_Hz           = 0,
  MAG_100_Hz         = 1
} mag_speed;


typedef enum
{
	DLPF_BANDWIDTH_184HZ,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
}DlpfBandwidth;

typedef enum
{
	ACCEL_RANGE_2G,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
}AccelRange;

typedef enum
{
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
}GyroRange;

typedef enum
{
  Successful          = 0,
  Failed         = 1
} Result_MPU9250;

void Init_MPU9250(void);
uint8_t I2C_WRITE_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t Value);

uint8_t I2C_READ_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value);

uint8_t I2C_READ_MULT_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value, uint8_t bytes);

void ReadGyro(float* GyroX, float* GyroY, float* GyroZ);

void ReadAcc(float* AccX, float* AccY, float* AccZ);

Result_MPU9250 ReadMag(float* MagX, float* MagY, float* MagZ);

void ReadFuseRomMag(float* MagX, float* MagY, float* MagZ);

void initAK8963(float* destination);

void InitIMU( I2C_ChannelType I2C);

status_t NewI2C_NC(uint8_t slaveAddress, i2c_direction_t I2C_Dir, uint8_t *volatile data, size_t datasize, uint8_t* subaddress, uint8_t subaddresssize);

Result_MPU9250 InitMpu9250Fast();

Result_MPU9250 Set_Accel_Range(accel_range range);

Result_MPU9250 Set_Gyro_Range(gyro_range range);

Result_MPU9250 set_mag_scale(mag_scale scale);

Result_MPU9250 set_mag_speed(mag_speed mspeed);

Result_MPU9250 CalibrateMagnetometer(void);

#endif /* MPU9250_H_ */
