/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    CodeToConvert.c
 * @brief   Application entry point.
 */
#include <I2C_Driver.h>
#include <I2C_Driver.h>
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "KalmanFilter.h"
#include "fsl_i2c.h"
#include "MadgwickAHRS.h"
#include "fsl_uart.h"
#include "fsl_pit.h"

#include "DataTypeDefinitions.h"
#include "GPIO.h"
#include "MPU9250.h"
#include "GlobalFunctions.h"

#define IMU 1
#define GPS 1
#define Kalman 1
#define IMU_CALIB 0
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
#define MSW_I2C_BAUD_RATE_200KHZ 0x16

#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define EXAMPLE_I2C_MASTER_BASEADDR I2C0

#define I2C_BAUDRATE 100000U

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART0
#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)
#define DEMO_UART_IRQn UART0_RX_TX_IRQn
#define DEMO_UART_IRQHandler UART0_RX_TX_IRQHandler

#define DEMO_UART3 UART3
#define DEMO_UART_CLKSRC3 UART3_CLK_SRC
#define DEMO_UART_CLK_FREQ3 CLOCK_GetFreq(UART3_CLK_SRC)
#define DEMO_UART_IRQn3 UART3_RX_TX_IRQn
#define DEMO_UART_IRQHandler3 UART3_RX_TX_IRQHandler


/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16

/*! @brief Ring buffer to save received data. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
float Hor_Lat;
float Min_Lat;
float Seg_Lat;
float Hor_Lon;
float Min_Lon;
float Seg_Lon;
volatile bool pitIsrFlag = false;

#define PIT_LED_HANDLER PIT0_IRQHandler
#define PIT_IRQ_ID PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

uint8_t g_tipString[] =
    "Uart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";

/*
  Ring buffer for data input and output, in this example, input data are saved
  to ring buffer in IRQ handler. The main function polls the ring buffer status,
  if there are new data, then send them out.
  Ring buffer full: (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) == txIndex)
  Ring buffer empty: (rxIndex == txIndex)
*/
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */
volatile uint16_t GPGGAInd = 0; /* Index of the memory to save new arrived data. */
/*******************************************************************************
 * Code
 ******************************************************************************/
uint8_t actual_data;
uint8_t GPGGA_Buffer[60] = {0};
uint8_t GPGSA[5]="GPVTG";
uint8_t GPGGA[5]="GPGGA";
uint8_t SPEED[4];
uint8_t LATITUD[10] = {0};
uint8_t LONGITUD[11]= {0};
uint8_t ALTITUD[5]= {0};
uint8_t Flag_Speed = 0;
uint8_t Flag_LLA = 0;
uint8_t Flag_Kalman = 0;
uint8_t i = 0;
uint8_t iF = 0;
uint8_t FlagIndex=0;
uint8_t FlagIndexLLA=0;

float Longitude, Latitude;
static void ConvertToLLA();
void ConvertToLLA()
{
	double LatMin, LatSec, LatDay, LongMin, LongSec, LongDay;

	LatDay = ((GPGGA_Buffer[11]-48) * 10.0f) + ((GPGGA_Buffer[12]-48));
	LatMin = ((double)(GPGGA_Buffer[13]-48) * 10.0f) + ((double)(GPGGA_Buffer[14]-48));
	LatSec = ((double)(GPGGA_Buffer[16]-48) * 10.0f) + ((double)(GPGGA_Buffer[17]-48));
	LatSec += ((double)(GPGGA_Buffer[18]-48) / 10.0f);
	LatSec += ((double)(GPGGA_Buffer[19]-48) / 100.0f);
	LatSec += ((double)(GPGGA_Buffer[20]-48) / 1000.0f);

	Latitude = LatDay + (LatMin/60.0f) + (LatSec/3600.0f);

	LongDay = ((GPGGA_Buffer[24]-48) * 100.0f) + ((GPGGA_Buffer[25]-48) * 10.0f) + ((GPGGA_Buffer[26]-48));
	LongMin = ((double)(GPGGA_Buffer[27]-48) * 10.0f) + ((double)(GPGGA_Buffer[28]-48));
	LongSec = ((double)(GPGGA_Buffer[30]-48) * 10.0f) + ((double)(GPGGA_Buffer[31]-48));
	LongSec += ((double)(GPGGA_Buffer[32]-48) / 10.0f);
	LongSec += ((double)(GPGGA_Buffer[33]-48) / 100.0f);
	LongSec += ((double)(GPGGA_Buffer[34]-48) / 1000.0f);

	Longitude = -(LongDay + (LongMin/60.0f) + (LongSec/3600.0f));

	/*LongDegres = ((GPGGA_Buffer[24]-48) * 100) + ((GPGGA_Buffer[25]-48) * 10) + ((GPGGA_Buffer[26]-48));
	LongSec = ((float)(GPGGA_Buffer[28]-48) * 10.0f) + ((float)(GPGGA_Buffer[29]-48)) + ((float)(GPGGA_Buffer[30]-48) / 10.0f) + ((float)(GPGGA_Buffer[31]-48) / 100.0f)
			+ ((float)(GPGGA_Buffer[32]-48) / 1000.0f) + ((float)(GPGGA_Buffer[33]-48) / 10000.0f) + ((float)(GPGGA_Buffer[34]-48) / 100000.0f);

	Longitude = LongDegres + (LongSec/60.0f);*/
}

void DEMO_UART_IRQHandler3(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEMO_UART3))
    {
        data = UART_ReadByte(DEMO_UART3);

        /* If ring buffer is not full, add data to ring buffer. */
        if (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) != txIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= DEMO_RING_BUFFER_SIZE;
            if (Flag_LLA)
            {
            	GPGGA_Buffer[GPGGAInd] = data;
            	GPGGAInd ++;
            	if (GPGGAInd == 60)
            	{
            		GPGGAInd = 0;
            		Flag_LLA = 0;
            		Flag_Kalman = 1;
            	}
            }
        }


        if(data == GPGGA[iF])
        {
        	iF+=1;
        	if(iF==5)
        	{
        			Flag_LLA=1;
        			FlagIndexLLA=(FlagIndexLLA+8)%DEMO_RING_BUFFER_SIZE;
        			iF=0;
        	}
        }else
        {
        	iF=0;
        }

    }

    /*Directivas de compilación*/

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void PIT_LED_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    pitIsrFlag = true;
    __DSB();
}

/*!
 * @brief Main function
 */

int main(void)
{
	uart_config_t config;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    i2c_master_config_t masterConfig;
    uint32_t sourceClock;
    pit_config_t pitConfig;
	float AccX_F;
	float AccY_F;
	float AccZ_F;
	float GyroX_F;
	float GyroY_F;
	float GyroZ_F;
	float MagX_F;
	float MagY_F;
	float MagZ_F;
	float destination[3] = {0};

	/*LLA2NED(-103.4174,20.6067, 1584.849);
	KalmanFilter(.2, 5.1528, 0.4789, -0.6312, 0, 0);
	KalmanFilter(.2, 5.2031, 0.4741, -0.6312, 0, 0);*/

	GPIO_pinControlRegisterType MUXAlt5 = GPIO_MUX5;
	GPIO_clockGating(GPIO_E);
	GPIO_pinControlRegister(GPIO_E,BIT24,&MUXAlt5);
	GPIO_pinControlRegister(GPIO_E,BIT25,&MUXAlt5);
#if (IMU)
	/**Initializes the I2C*/
	I2C_init(I2C_1,400000,250);

    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = I2C_MASTER_CLK_FREQ;

    I2C_MasterInit(EXAMPLE_I2C_MASTER_BASEADDR, &masterConfig, sourceClock);
     InitMpu9250Fast();

     Set_Accel_Range(RANGE_4G);

     Set_Gyro_Range(RANGE_GYRO_250);

     set_mag_scale(SCALE_14_BITS);

     set_mag_speed(MAG_100_Hz);

     /*PIT INITIALIZATION*/
     PIT_GetDefaultConfig(&pitConfig);

     /* Init pit module */
     PIT_Init(PIT, &pitConfig);

     /* Set timer period for channel 0 */
     PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(10000, PIT_SOURCE_CLOCK));

     /* Enable timer interrupts for channel 0 */
     PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
     //FXOS8700CQ_Accel_Calibration(FXOS8700drv);
     /* Enable at the NVIC */

	float Roll, Pitch, Yaw;
	ReadMag(&MagX_F, &MagY_F, &MagZ_F);
#endif
#if (GPS)
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
	config.enableTx = true;
	config.enableRx = true;
	config.baudRate_Bps = 9600;
	UART_Init(DEMO_UART3, &config, DEMO_UART_CLK_FREQ3);
	UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
	config.baudRate_Bps = 9600;
	UART_Init(DEMO_UART3, &config, DEMO_UART_CLK_FREQ3);
	UART_EnableInterrupts(DEMO_UART3, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
	EnableIRQ(DEMO_UART_IRQn3);
    EnableIRQ(PIT_IRQ_ID);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
#endif
#if (IMU_CALIB)
	CalibrateMagnetometer();
#endif
	Result_MPU9250 Result;
	float N, E, D;
	while (1)
	{
		/*if (pitIsrFlag)
		{
			ReadMag(&MagX_F, &MagY_F, &MagZ_F);
			ReadGyro(&GyroX_F, &GyroY_F, &GyroZ_F);
			ReadAcc(&AccX_F, &AccY_F, &AccZ_F);
			MadgwickAHRSupdate(GyroX_F, -GyroY_F, -GyroZ_F, AccX_F, -AccY_F, AccZ_F, MagY_F, -MagX_F,MagZ_F, &Roll, &Pitch, &Yaw);
			pitIsrFlag = 0;
		}

#if (GPS)
		if (Flag_Kalman)
		{
			ConvertToLLA();
			printf("%f %f \n", Latitude, Longitude);
			Flag_Kalman = 0;
		}
		while ((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(DEMO_UART)) && (rxIndex != txIndex))
		{
			UART_WriteByte(DEMO_UART, demoRingBuffer[txIndex]);

			txIndex++;
			txIndex %= DEMO_RING_BUFFER_SIZE;

		}
#endif*/
#if (Kalman)
		 if(pitIsrFlag)
		 {
			 pitIsrFlag = false;
			 Result = ReadMag(&MagX_F, &MagY_F, &MagZ_F);
			 ReadGyro(&GyroX_F, &GyroY_F, &GyroZ_F);
			 ReadAcc(&AccX_F, &AccY_F, &AccZ_F);
			MadgwickAHRSupdate(GyroZ_F, GyroY_F, GyroX_F, AccZ_F, AccY_F, AccX_F, MagZ_F, MagY_F,MagX_F, &Roll, &Pitch, &Yaw);
		 }
		 if(Flag_Kalman)
		 {
			 ConvertToLLA();
			 LLA2NED(Longitude,Latitude, 1584.849, &N, &E, &D);
			 KalmanFilter(1, Yaw, AccX_F, N, E, D);
		 }
#endif
	}
    return 0;
}


