/*
 * MPU9250.c
 *
 *  Created on: 29/11/2019
 *      Author: loya_
 */

#define DELAYI2C 40

#include "DataTypeDefinitions.h"
#include <I2C_Driver.h>
#include "GlobalFunctions.h"
#include "GPIO.h"
#include "MPU9250.h"
#include "fsl_i2c.h"

#define EXAMPLE_I2C_MASTER_BASEADDR I2C0
#define DATE_READ 				0x01


#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

const float G = 9.807f;
float _gyroScale;

const float _d2r = 3.14159265359f/180.0f;
AccelRange _accelRange;
GyroRange _gyroRange;
DlpfBandwidth _bandwidth;
uint8_t _buffer[21];
uint8 result[20];
float _magScaleX, _magScaleY, _magScaleZ;

uint32_t flags;            /*!< A transfer flag which controls the transfer. */
uint8_t slaveAddress;      /*!< 7-bit slave address. */
i2c_direction_t direction; /*!< A transfer direction, read or write. */
uint32_t subaddress;       /*!< A sub address. Transferred MSB first. */
uint8_t subaddressSize;    /*!< A size of the command buffer. */
uint8_t *volatile data;    /*!< A transfer buffer. */
volatile size_t dataSize;  /*!< A transfer size. */

const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t ACCEL_DLPF_184 = 0x01;
const uint8_t ACCEL_DLPF_92 = 0x02;
const uint8_t ACCEL_DLPF_41 = 0x03;
const uint8_t ACCEL_DLPF_20 = 0x04;
const uint8_t ACCEL_DLPF_10 = 0x05;
const uint8_t ACCEL_DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_DLPF_184 = 0x01;
const uint8_t GYRO_DLPF_92 = 0x02;
const uint8_t GYRO_DLPF_41 = 0x03;
const uint8_t GYRO_DLPF_20 = 0x04;
const uint8_t GYRO_DLPF_10 = 0x05;
const uint8_t GYRO_DLPF_5 = 0x06;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;
// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;

status_t NewI2C_NC(uint8_t slaveAddress, i2c_direction_t I2C_Dir, uint8_t *volatile data, size_t datasize, uint8_t* subaddress, uint8_t subaddresssize)
{
	i2c_master_transfer_t masterXfer;
	status_t Value = 0;
    masterXfer.slaveAddress = slaveAddress;
    masterXfer.direction = I2C_Dir;
    masterXfer.subaddress = (uint32_t)(*subaddress);
    masterXfer.subaddressSize = subaddresssize;
    masterXfer.data = data;
    masterXfer.dataSize = datasize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    Value = I2C_MasterTransferBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &masterXfer);
    return Value;
}

uint8_t I2C_WRITE_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t Value)
{
	/*This function initiliazes the RTC by writing in the ST in the seconds byte
		 * Before that it has to be in TX mode and it has to specify the I2C to be used as a slave.
		 * Once it has done all its processing, it checks if the oscillator is running and wait until it's stopped*/
	I2C_TX_RX_Mode(TRUE, I2C);

		/** Sends a start signal*/
	I2C_start(I2C);

		/** Writes the RTC address, in write mode, and waits for the ACK*/
	I2C_write_Byte(SlaveAddress, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

		/** Writes the seconds address and waits for the ACK*/
	I2C_write_Byte(Register, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

	I2C_write_Byte(Value, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

		/** Stop signal*/
	I2C_stop(I2C);

	return 0;
}

uint8_t I2C_READ_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value){
	/** Return variable*/
	//uint8 Val;
	/** Prepares the I2C to transmit*/
	I2C_TX_RX_Mode(TRUE, I2C);


	/** Sends a start signal*/
	I2C_start(I2C);

	/** Writes the RTC address, in write mode, and waits for the ACK*/
	I2C_write_Byte((SlaveAddress), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

	delay(DELAYI2C);

	/** Writes the seconds address and waits for the ACK*/
	I2C_write_Byte(Register, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Sends a repeated start*/
	I2C_repeted_Start(I2C);

	/** Writes the RTC address, in read mode, and waits for the ACK*/
	I2C_write_Byte(((SlaveAddress) | 1), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Prepares the I2C to receive*/
	I2C_TX_RX_Mode(FALSE, I2C);

	/** Prepares the NACK*/
	I2C_NACK(I2C);
	/** Dummy read*/
	*Value = I2C_read_Byte(I2C);
	I2C_wait(I2C);
	delay(DELAYI2C);

	/** Stop signal*/
	I2C_stop(I2C);
	/** Real read*/
	*Value = I2C_read_Byte(I2C);
	delay(DELAYI2C);
	return (1);
}

uint8_t I2C_READ_MULT_MPU(I2C_ChannelType I2C, uint8_t SlaveAddress, uint8_t Register, uint8_t* Value, uint8_t bytes){
	/** Return variable*/

	uint8_t i = 0;
	/** Prepares the I2C to transmit*/
	I2C_TX_RX_Mode(TRUE, I2C);


	/** Sends a start signal*/
	I2C_start(I2C);

	/** Writes the RTC address, in write mode, and waits for the ACK*/
	I2C_write_Byte((SlaveAddress), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);

	delay(DELAYI2C);

	/** Writes the seconds address and waits for the ACK*/
	I2C_write_Byte(Register, I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Sends a repeated start*/
	I2C_repeted_Start(I2C);

	/** Writes the RTC address, in read mode, and waits for the ACK*/
	I2C_write_Byte(((SlaveAddress) | 1), I2C);
	I2C_wait(I2C);
	I2C_get_ACK(I2C);
	delay(DELAYI2C);

	/** Prepares the I2C to receive*/
	I2C_TX_RX_Mode(FALSE, I2C);

	/** Dummy read*/
	*Value = I2C_read_Byte(I2C);
	I2C_wait(I2C);
	delay(DELAYI2C);

	for(i = 0; i < bytes - 2;i++)
	{
		*(Value + i) = I2C_read_Byte(I2C);
		I2C_wait(I2C);
		delay(DELAYI2C);
	}
	/** Prepares the NACK*/
	I2C_NACK(I2C);
	*(Value + i++) = I2C_read_Byte(I2C);
	I2C_wait(I2C);
	delay(DELAYI2C);
	/** Stop signal*/
	I2C_stop(I2C);
	/** Real read*/
	*(Value + i++) = I2C_read_Byte(I2C);

	return (1);
}

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
#define PWR_MGMT_2       0x6C
void Init_MPU9250(void){

	//I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, ACC_FULL_SCALE_16_G);
	//I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0, 0);
	//I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, GYRO_FULL_SCALE_2000_DPS);
	//Enable ByPass
	//I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, 0x0A, 0x1F);
	//uint8_t ReadMPU9250[2] = {0};
	uint8_t WriteNew[2] = {0b10000000,0};
	uint8_t  writeNew= PWR_MGMNT_1;
	NewI2C_NC(MPU9250_ADDR, kI2C_Write, WriteNew, 1, &writeNew, 1);

	WriteNew[0] = 0x00;
	writeNew= PWR_MGMT_2;
	NewI2C_NC(MPU9250_ADDR, kI2C_Write, WriteNew, 1, &writeNew, 1);

	WriteNew[0] = 0x02;
	writeNew= INT_PIN_CFG;
	NewI2C_NC(MPU9250_ADDR, kI2C_Write, WriteNew, 1, &writeNew, 1);
	// set AK8963 to Power Down
	WriteNew[0] = 0;
	writeNew= 0x00;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, WriteNew, 1, &writeNew, 1);

	WriteNew[0] = 0;
	writeNew= AK8963_CNTL1;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, WriteNew, 1, &writeNew, 1);

	WriteNew[0] = 0x0F;
	writeNew= AK8963_CNTL1;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, WriteNew, 1, &writeNew, 1);

	uint8_t Read[3] = {0};
	writeNew= 0x10;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, Read, 3, &writeNew, 1);

	WriteNew[0] = 0;
	writeNew= AK8963_CNTL1;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, WriteNew, 1, &writeNew, 1);
	delay(100000);
	WriteNew[0] = 0b00010010;
	writeNew= AK8963_CNTL1;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, WriteNew, 1, &writeNew, 1);


	/*
	 *     // Read factory ajustment data
  __write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
  delay(10);
  // Enter Fuse ROM access mode
  __write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
  delay(10);
	 *   // Reset the internal registers and restores the default settings.
  __write_byte(MPU9250_ADDRESS, PWR_MGMT_1, 0b10000000);

  delay(100);

  // Activate all sensors
  __write_byte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);


  //Mag startup
  // Activate bypass to access Magnetometer
  __write_byte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);

  // Verify ID
  if (__read_byte(AK8963_ADDRESS, WHO_AM_I_AK8963) != WHO_AM_I_RESP_MAG)
	 * uint8_t WriteMPU9250[2] = {0x2,0};
	uint8_t  write= 0x37;
	WriteMPU9250[0] = ACC_FULL_SCALE_16_G;
	write= 0x1C;
	NewI2C_NC(MPU9250_ADDR, kI2C_Write, WriteMPU9250, 1, &write, 1);

	WriteMPU9250[0] = GYRO_FULL_SCALE_2000_DPS;
	write= 0x1B;
	NewI2C_NC(MPU9250_ADDR, kI2C_Write, WriteMPU9250, 1, &write, 1);

	WriteMPU9250[0] = 0x22;
	write= 0x37;
	NewI2C_NC(MPU9250_ADDR, kI2C_Write, WriteMPU9250, 1, &write, 1);

	WriteMPU9250[0] = 0x01;
	write= 0x0A;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, WriteMPU9250, 1, &write, 1);*/

/*
 * 	// Configurar acelerometro
	I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
	// Configurar giroscopio
	I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
	// Configurar magnetometro
	I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
	I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
 */

	/*I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, 0x6A, 0x00);
	ReadMPU9250 = 0;
	I2C_READ_MPU(I2C_0, MPU9250_ADDR, WHO_AM_I, &ReadMPU9250);
	I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, 0x37, 0x22);
	ReadMPU9250 = 0;
	I2C_READ_MPU(I2C_0, MPU9250_ADDR, WHO_AM_I, &ReadMPU9250);*/
	//I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, 0x38, 0x01);
	/*uint8_t enable2;
	I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x00, &enable2);
	I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, 0x0A, 0x1F);
	delay(5000);*/
	//I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, PIN_CONFIG, 0x22);
	//I2C_WRITE_MPU(I2C_0, MPU9250_ADDRESS, 0x38, 0x01);
	//uint8_t enable;
	//I2C_READ_MPU(I2C_0, MPU9250_ADDRESS, PIN_CONFIG, &enable);

	//I2C_READ_MPU(I2C_0, MAG_ADDRESS, PIN_CONFIG, &enable);
	//I2C_READ_MPU(I2C_0, MAG_ADDRESS, 0x00, &enable2);
	//I2C_WRITE_MPU(I2C_0, MAG_ADDRESS, MPU9250_MAG_CONFIG, 0x01);
}

const int16_t tX[3] = {0,  1,  0};
const int16_t tY[3] = {1,  0,  0};
const int16_t tZ[3] = {0,  0, -1};

const float _accelScale = 9.807f * 16.0f/32767.5f; // setting the accel scale to 16G
//_accelRange = ACCEL_RANGE_16G;


void ReadAcc(float* AccX, float* AccY, float* AccZ)
{
	uint8_t ReadMPU9250[6] = {0};
	uint8_t  write= 0x3B;
	int16_t AccX_16, AccY_16, AccZ_16;

	NewI2C_NC(MPU9250_ADDR, kI2C_Read, ReadMPU9250, 6, &write, 1);

	AccX_16 = (((int16_t)ReadMPU9250[0]<<8) | (ReadMPU9250[1]));
	AccY_16 = (((int16_t)ReadMPU9250[2]<<8) | (ReadMPU9250[3]));
	AccZ_16 = (((int16_t)ReadMPU9250[4]<<8) | (ReadMPU9250[5]));

	*AccX = ((float)(tX[0]*AccX_16 + tX[1]*AccY_16 + tX[2]*AccZ_16) * _accelScale);
	*AccY = ((float)(tY[0]*AccX_16 + tY[1]*AccY_16 + tY[2]*AccZ_16) * _accelScale);
	*AccZ = ((float)(tZ[0]*AccX_16 + tZ[1]*AccY_16 + tZ[2]*AccZ_16) * _accelScale);


	return;
}

void ReadGyro(float* GyroX, float* GyroY, float* GyroZ)
{
	uint8_t ReadMPU9250[6] = {0};
	uint8_t  write= 0x43;
	int16_t GyroX_16, GyroY_16, GyroZ_16;
	_gyroScale = 2000.0f/32767.5f * _d2r;

	NewI2C_NC(MPU9250_ADDR, kI2C_Read, ReadMPU9250, 6, &write, 1);

	GyroX_16 = ((int16_t)ReadMPU9250[0]<<8) | (ReadMPU9250[1]);
	GyroY_16 = ((int16_t)ReadMPU9250[2]<<8) | (ReadMPU9250[3]);
	GyroZ_16 = ((int16_t)ReadMPU9250[4]<<8) | (ReadMPU9250[5]);

	*GyroX = ((float)(tX[0]*GyroX_16 + tX[1]*GyroY_16 + tX[2]*GyroZ_16) * _gyroScale);
	*GyroY = ((float)(tY[0]*GyroX_16 + tY[1]*GyroY_16 + tY[2]*GyroZ_16) * _gyroScale);
	*GyroZ = ((float)(tZ[0]*GyroX_16 + tZ[1]*GyroY_16 + tZ[2]*GyroZ_16) * _gyroScale);
	return;
}

void ReadFuseRomMag(float* MagX, float* MagY, float* MagZ)
{
	uint8_t ReadMPU9250[3] = {0};
	uint8_t  write= 0x10;

	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, ReadMPU9250, 3, &write, 1);

	*MagX = -((int16_t)ReadMPU9250[1]<<8) | (ReadMPU9250[0]);

	*MagY = -((int16_t)ReadMPU9250[3]<<8) | (ReadMPU9250[2]);

	*MagZ = ((int16_t)ReadMPU9250[5]<<8) | (ReadMPU9250[4]);
	return;
}

void initAK8963(float* destination)
{
// First extract the factory calibration for each magnetometer axis
uint8_t rawData[3];  // x/y/z gyro calibration data stored here
// Power down magnetometer
I2C_WRITE_MPU(I2C_0, AK8963_I2C_ADDR, 0x0A, 0x00);
delay(2000);
I2C_WRITE_MPU(I2C_0, AK8963_I2C_ADDR, 0x0A, 0x0F);
delay(2000);
I2C_READ_MPU(I2C_0, AK8963_I2C_ADDR, 0x10, &rawData[0]);
delay(2000);
I2C_READ_MPU(I2C_0, AK8963_I2C_ADDR, 0x11, &rawData[1]);
delay(2000);
I2C_READ_MPU(I2C_0, AK8963_I2C_ADDR, 0x12, &rawData[2]);
delay(2000);
destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
I2C_WRITE_MPU(I2C_0, AK8963_I2C_ADDR, 0x0A, 0x00);
delay(2000);
// Configure the magnetometer for continuous read and highest resolution
// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
I2C_WRITE_MPU(I2C_0, AK8963_I2C_ADDR, 0x0A, 1 << 4 | 0x06);
delay(2000);
}

void InitIMU( I2C_ChannelType I2C)
{
	uint8_t ReadMPU9250;
	uint8_t ReadAK8963;
	I2C_WRITE_MPU(I2C, MPU9250_ADDR, PWR_MGMNT_1, CLOCK_SEL_PLL);

	I2C_WRITE_MPU(I2C, MPU9250_ADDR, PWR_MGMNT_1, I2C_MST_EN);

	I2C_WRITE_MPU(I2C, MPU9250_ADDR, PWR_MGMNT_1, I2C_MST_CLK);
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_WHO_AM_I, &ReadAK8963);
	// Set AK8963 to Power Down
	I2C_WRITE_MPU(I2C, AK8963_I2C_ADDR, AK8963_CNTL1, AK8963_PWR_DOWN);
	I2C_WRITE_MPU(I2C, MPU9250_ADDR, PWR_MGMNT_1, PWR_RESET);

	delay(50000);
	I2C_WRITE_MPU(I2C, AK8963_I2C_ADDR, AK8963_CNTL2, AK8963_RESET);
	I2C_WRITE_MPU(I2C, MPU9250_ADDR, PWR_MGMNT_1, CLOCK_SEL_PLL);

	I2C_READ_MPU(I2C, MPU9250_ADDR, WHO_AM_I, &ReadMPU9250);
	if(ReadMPU9250 != 0x71 && ReadMPU9250 != 0x73)
	{
		return;
	}
	// Enable Accelerometer and Gyroscope
	I2C_WRITE_MPU(I2C, MPU9250_ADDR, PWR_MGMNT_2, SEN_ENABLE);

	//Setting Accelerometer range 10 16G as default
	I2C_WRITE_MPU(I2C, MPU9250_ADDR, ACCEL_CONFIG, ACCEL_FS_SEL_16G);

//	_accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
//	_accelRange = ACCEL_RANGE_16G;
	// setting the gyro range to 2000DPS as default
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);

//	_gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
//	_gyroRange = GYRO_RANGE_2000DPS;
	// Setting Bandwith to 184 Hz as default
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, ACCEL_CONFIG2, ACCEL_DLPF_184);
	// Setting Gyroscope Bandwith to 184Hz
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, CONFIG, GYRO_DLPF_184);
//	_bandwidth = DLPF_BANDWIDTH_184HZ;
	// Setting the Sample Rate Divider to 0 as Default
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, SMPDIV, 0x00);
	//Enable I2C master mode
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, USER_CTRL, I2C_MST_EN);
	//Set the I2C Bus Speed to 400 kHz
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, I2C_MST_CTRL, I2C_MST_CLK);

//	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_WHO_AM_I, &ReadAK8963);
	/*if(ReadAK8963 != 0x72)
	{
		return;
	}*/
	//Get the magnetometer Calibration
	// Set AK8963 to Power Down
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, AK8963_CNTL1, AK8963_PWR_DOWN);
	// Long wait between AK9863 mode changes
//	delay(5000);
	// Set AK8963 to FUSE ROM acces
//	I2C_WRITE_MPU(I2C, MPU9250_ADDR, AK8963_CNTL1, AK8963_FUSE_ROM);
	//Long wait between AK9863 mode changes
	delay(5000);
	 // read the AK8963 ASA registers and compute magnetometer scale factors
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA, _buffer);
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 1, (_buffer + 1));
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 2, (_buffer + 2));

	_magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	// Set AK8963 to Power Down

	I2C_WRITE_MPU(I2C, AK8963_I2C_ADDR, AK8963_CNTL1, AK8963_PWR_DOWN);
	//Long wait between AK9863 mode changes
	delay(5000);
	I2C_WRITE_MPU(I2C, AK8963_I2C_ADDR, AK8963_CNTL1, AK8963_CNT_MEAS2);
	//Long wait between AK9863 mode changes
	delay(5000);
	//Select Clock Source to Gyro
	I2C_WRITE_MPU(I2C, MPU9250_ADDR, PWR_MGMNT_1, CLOCK_SEL_PLL);
	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 3, (_buffer + 3));
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 4, (_buffer + 4));
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 5, (_buffer + 5));
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 6, (_buffer + 6));
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 7, (_buffer + 7));
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 8, (_buffer + 8));
	I2C_READ_MPU(I2C, AK8963_I2C_ADDR , AK8963_ASA + 9, (_buffer + 9));
	//Estimate GYRO BIAS

	return;
}

#define WHO_AM_I_RESP    0x73 // MPU9250 is 0x71 and MPU9255 is 0x73
//Accel and Gyro registers
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define TEMP_OUT_H       0x41
#define GYRO_XOUT_H      0x43
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define WHO_AM_I         0x75
#define MPU9250_ADDRESS 0x68

//Magnetometer Registers
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define AK8963_ST1       0x02  // data status
#define AK8963_XOUT_L    0x03  // data
#define AK8963_CNTL      0x0A  // Power down (0000), Continuous measurement mode 1 (0010), CMM2 (0110), Fuse ROM (1111) on bits 3:0
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
#define WHO_AM_I_RESP_MAG 0x48

#define AK8963_ADDRESS   0x0C
float MagAdjustment[3] = {0, 0, 0};
Result_MPU9250 InitMpu9250Fast()
{
	uint8_t Read[2] = {0, 0};
	uint8_t WriteReg = WHO_AM_I;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Read, Read, 1, &WriteReg, 1);
	if (Read[0] != WHO_AM_I_RESP)
		return Failed;

	// Reset the internal register
	Read[0] = 0b10000000;
	WriteReg = PWR_MGMT_1;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);

	//Activate all sensors
	Read[0] = 0x00;
	WriteReg = PWR_MGMT_2;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);

	//Mag Startup
	//Activate bypass to access Magnetometer
	Read[0] = 0x02;
	WriteReg = INT_PIN_CFG;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);

	//Verify ID
	Read[0] = 0x00;
	WriteReg = WHO_AM_I_AK8963;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, Read, 1, &WriteReg, 1);

	if (Read[0] != WHO_AM_I_RESP_MAG)
		return Failed;

	//Read Factory adjustment data
	Read[0] = 0x00;
	WriteReg = AK8963_CNTL;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, Read, 1, &WriteReg, 1);

	//Enter Fuse ROM access mode
	Read[0] = 0x0F;
	WriteReg = AK8963_CNTL;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, Read, 1, &WriteReg, 1);

	// Read the x-, y-, and z-axis calibration values
	uint8_t MagRead[3] = {0};
	WriteReg = AK8963_ASAX;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, MagRead, 3, &WriteReg, 1);
	MagAdjustment[0] =  (float)(MagRead[0] - 128)/256.0f + 1.0f;
	MagAdjustment[1] =  (float)(MagRead[1] - 128)/256.0f + 1.0f;
	MagAdjustment[2] =  (float)(MagRead[2] - 128)/256.0f + 1.0f;

	// Return x-axis sensitivity adjustment values, etc.
	Read[0] = 0x00;
	WriteReg = AK8963_CNTL;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, Read, 1, &WriteReg, 1);

	// Return x-axis sensitivity adjustment values, etc.
	Read[0] = 0b00010110;
	WriteReg = AK8963_CNTL;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Write, Read, 1, &WriteReg, 1);

	return Successful;
}

Result_MPU9250 Set_Accel_Range(accel_range range)
{
	uint8_t Read[2] = {0, 0};
	uint8_t WriteReg = ACCEL_CONFIG;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Read, Read, 1, &WriteReg, 1);

	Read[0] = ((Read[0] & 0b11100111)|(range<<3));
	 WriteReg = ACCEL_CONFIG;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);

	return Successful;
}
#define GYRO_CONFIG      0x1B

Result_MPU9250 Set_Gyro_Range(gyro_range range)
{
	uint8_t Read[2] = {0, 0};
	uint8_t WriteReg = GYRO_CONFIG;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Read, Read, 1, &WriteReg, 1);

	Read[0] = ((Read[0] & 0b11100111)|(range<<3));
	WriteReg = GYRO_CONFIG;
	NewI2C_NC(MPU9250_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);

	return Successful;
}

Result_MPU9250 set_mag_scale(mag_scale scale)
{
	uint8_t Temporal;
	uint8_t Read[2] = {0, 0};
	uint8_t WriteReg = AK8963_CNTL;
	// Save mag configs
	NewI2C_NC(AK8963_ADDRESS, kI2C_Read, Read, 1, &WriteReg, 1);

	Temporal = (Read[0] & 0b00000110);
	// disable mag
	Read[0] = 0;
	 WriteReg = AK8963_CNTL;
	NewI2C_NC(AK8963_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);
	Read[0] = (scale << 4) | Temporal;
	 WriteReg = AK8963_CNTL;
	NewI2C_NC(AK8963_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);

	return Successful;
}

Result_MPU9250 set_mag_speed(mag_speed mspeed)
{
	uint8_t Temporal;
	uint8_t Read[2] = {0, 0};
	uint8_t WriteReg = AK8963_CNTL;
	// Save mag configs
	NewI2C_NC(AK8963_ADDRESS, kI2C_Read, Read, 1, &WriteReg, 1);

	// disable mag
	Read[0] = 0;
	WriteReg = AK8963_CNTL;
	NewI2C_NC(AK8963_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);
	if(mspeed==MAG_8_Hz)
	{
		Read[0] = (Read[0] | 0b0010);
		WriteReg = AK8963_CNTL;
		NewI2C_NC(AK8963_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);
	}
	else
	{
		Read[0] = (Read[0] | 0b0110);
		WriteReg = AK8963_CNTL;
		NewI2C_NC(AK8963_ADDRESS, kI2C_Write, Read, 1, &WriteReg, 1);
	}

	return Successful;

}

float scale_x=0.233333334,scale_y=-0.10510511,scale_z=0.121527784;
float offset_x=-25,offset_y=55.5,offset_z=-48;

typedef struct
{
    uint32_t timestamp; /*! The time, this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
    int16_t mag[3];     /*!< The mag data */
} accelmagdata_t;

Result_MPU9250 CalibrateMagnetometer(void)
{
    float Xout_Mag_16_bit_avg, Yout_Mag_16_bit_avg, Zout_Mag_16_bit_avg;
    int16_t Xout_Mag_16_bit_max, Yout_Mag_16_bit_max, Zout_Mag_16_bit_max;
    int16_t Xout_Mag_16_bit_min, Yout_Mag_16_bit_min, Zout_Mag_16_bit_min;
    uint16_t i = 0;
    uint8_t  writeStatus= AK8963_ST1;
    uint8_t  write= 0x03;
    uint8_t dataReady;
    uint8_t ReadMPU9250[7] = {0};

    accelmagdata_t Mag_16_bit;
    while(i < 3500)
    {
    	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, &dataReady, 1, &writeStatus, 1);
    	if ((dataReady & 0x01))
    	{
    		NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, ReadMPU9250, 7, &write, 1);
    		if(!(ReadMPU9250[6] & 0x08))
    		{
    			Mag_16_bit.mag[0] = (int16_t)(((int16_t)ReadMPU9250[1]<<8) | (ReadMPU9250[0]));

    			Mag_16_bit.mag[1] = (int16_t)(((int16_t)ReadMPU9250[3]<<8) | (ReadMPU9250[2]));

    			Mag_16_bit.mag[2] = (int16_t)(((int16_t)ReadMPU9250[5]<<8) | (ReadMPU9250[4]));
    		}

    		if (i == 0)
    		{
    			Xout_Mag_16_bit_max = Mag_16_bit.mag[0];
    			Xout_Mag_16_bit_min = Mag_16_bit.mag[0];

    			Yout_Mag_16_bit_max = Mag_16_bit.mag[1];
    			Yout_Mag_16_bit_min = Mag_16_bit.mag[1];

    			Zout_Mag_16_bit_max = Mag_16_bit.mag[2];
    			Zout_Mag_16_bit_min = Mag_16_bit.mag[2];
    		}
    		 if (Mag_16_bit.mag[0] > Xout_Mag_16_bit_max && Mag_16_bit.mag[0]!=0)
    		 {
    			 Xout_Mag_16_bit_max = Mag_16_bit.mag[0];
    		 }

    		 if (Mag_16_bit.mag[0] < Xout_Mag_16_bit_min && Mag_16_bit.mag[0]!=0)
    		 {
    			 Xout_Mag_16_bit_min = Mag_16_bit.mag[0];
    		 }

             if (Mag_16_bit.mag[1] > Yout_Mag_16_bit_max && Mag_16_bit.mag[1]!= 0)
             {
            	 Yout_Mag_16_bit_max = Mag_16_bit.mag[1];
             }

             if (Mag_16_bit.mag[1] < Yout_Mag_16_bit_min && Mag_16_bit.mag[1]!=0)
             {
            	 Yout_Mag_16_bit_min = Mag_16_bit.mag[1];
             }



             // Check to see if current sample is the maximum or minimum Z-axis value

             if (Mag_16_bit.mag[2] > Zout_Mag_16_bit_max && Mag_16_bit.mag[2]!=0)
             {
            	 Zout_Mag_16_bit_max = Mag_16_bit.mag[2];
             }

             if (Mag_16_bit.mag[2] < Zout_Mag_16_bit_min && Mag_16_bit.mag[2]!=0)
             {
            	 Zout_Mag_16_bit_min = Mag_16_bit.mag[2];
             }
             delay(2000);
             i++;
    	}
    }

    Xout_Mag_16_bit_avg = (float)((float)Xout_Mag_16_bit_max + (float)Xout_Mag_16_bit_min) / 2.0f;            // X-axis hard-iron offset
    Yout_Mag_16_bit_avg = (float)((float)Yout_Mag_16_bit_max + (float)Yout_Mag_16_bit_min) / 2.0f;            // Y-axis hard-iron offset
    Zout_Mag_16_bit_avg = (float)((float)Zout_Mag_16_bit_max + (float)Zout_Mag_16_bit_min) / 2.0f;            // Z-axis hard-iron offset

    offset_x =Xout_Mag_16_bit_avg;
    offset_y =Yout_Mag_16_bit_avg;
    offset_z =Zout_Mag_16_bit_avg;

    float avg_delta;
    avg_delta = (float)(Xout_Mag_16_bit_avg+Yout_Mag_16_bit_avg+Zout_Mag_16_bit_avg)/3.0f;
    scale_x=avg_delta/(float)Xout_Mag_16_bit_avg;
    scale_y=avg_delta/(float)Yout_Mag_16_bit_avg;
    scale_z=avg_delta/(float)Zout_Mag_16_bit_avg;
    return Successful;
}

Result_MPU9250 ReadMag(float* MagX, float* MagY, float* MagZ)
{
	uint8_t ReadStatus[2] = {0};
	uint8_t  writeStatus= AK8963_ST1;
	int16_t mx, my, mz;
	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, ReadStatus, 1, &writeStatus, 1);

	while(!(ReadStatus[0] & 0x01))
	{
		writeStatus= 0x02;
		NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, ReadStatus, 1, &writeStatus, 1);
	}

	uint8_t ReadMPU9250[7] = {0};
	uint8_t  write= 0x03;

	NewI2C_NC(AK8963_I2C_ADDR, kI2C_Read, ReadMPU9250, 7, &write, 1);
	if(!(ReadMPU9250[6] & 0x08))
	{
		mx = (int16_t)(((int16_t)ReadMPU9250[1]<<8) | (ReadMPU9250[0]));

		my = (int16_t)(((int16_t)ReadMPU9250[3]<<8) | (ReadMPU9250[2]));

		mz = (int16_t)(((int16_t)ReadMPU9250[5]<<8) | (ReadMPU9250[4]));

		*MagX = (float)((mx-offset_x)*scale_x)*10.0*4219.0/8190.0*MagAdjustment[0];
		*MagY = (float)((my-offset_y)*scale_y)*10.0*4219.0/8190.0*MagAdjustment[1];
		*MagZ = (float)((mz-offset_z)*scale_z)*10.0*4219.0/8190.0*MagAdjustment[2];
	}else
	{
		return Failed;
	}
/*
	*MagX = (float)mx*4912.0f*MagAdjustment[0] / 32760.0f;
	*MagY = (float)my*4912.0f*MagAdjustment[1] / 32760.0f;
	*MagZ = (float)mz*4912.0f*MagAdjustment[2] / 32760.0f;*/
	return Successful;
}
