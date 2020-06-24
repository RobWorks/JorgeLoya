/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file fxas21002_interrupt.c
 * @brief The fxas21002_interrupt.c file implements the ISSDK FXAS21002 sensor
 *        driver example demonstration with interrupt mode.
 */

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "board.h"
#include "GPIO.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include <math.h>
#include "fsl_pit.h"
#include "MadgwickAHRS.h"
//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C.h"
//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "fxas21002_drv.h"
#include "fxos8700_drv.h"

// UART Declaration
#define DEMO_UART UART0
#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)

#define PIT_LED_HANDLER PIT0_IRQHandler
#define PIT_IRQ_ID PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()

#define CALIBRATE_MAGNETOMETER (0)
#define MAGNETOMETER_CALIB (0)
#define MAGNETOMETER_CALIBON (0)
#define MAGNETOMETER_RAY_TEST (0)

/*Directivas de compilación*/
volatile bool pitIsrFlag = false;

//float scale_x=1.13461542,scale_y=1.09767437,scale_z=0.828070164;
//float offset_x=416,offset_y=430,offset_z=570;

float scale_x=0.962217,scale_y=0.962217,scale_z=1.079096;
float offset_x=726,offset_y=319,offset_z=1110;

float X_Accel_offset=180, Y_Accel_offset=181, Z_Accel_offset=182;
float GainX, GainY, GainZ;

float sstatex = 0;
float sstatey = 0;
float sstatez = 0;

float MagneticFieldX = 0;
float MagneticFieldY = 0;
float MagneticFieldZ = 0;
//-----------------------------------------------------------------------
// Constants
float Heading;
//-----------------------------------------------------------------------
/*! Prepare the register write list to configure FXAS21002 in non-FIFO mode. */
const registerwritelist_t fxas21002_Config_Isr[] = {
    /*! Configure CTRL_REG1 register to put FXAS21002 to 12.5Hz sampling rate. */
    {FXAS21002_CTRL_REG1, FXAS21002_CTRL_REG1_DR_200HZ, FXAS21002_CTRL_REG1_DR_MASK},
    /*! Configure CTRL_REG2 register to set interrupt configuration settings. */
    {FXAS21002_CTRL_REG2, FXAS21002_CTRL_REG2_IPOL_ACTIVE_HIGH | FXAS21002_CTRL_REG2_INT_EN_DRDY_ENABLE |
                              FXAS21002_CTRL_REG2_INT_CFG_DRDY_INT1,
     FXAS21002_CTRL_REG2_IPOL_MASK | FXAS21002_CTRL_REG2_INT_EN_DRDY_MASK | FXAS21002_CTRL_REG2_INT_CFG_DRDY_MASK},
    __END_WRITE_DATA__};

/*! Prepare the register read list to read FXAS21002 DataReady status. */
const registerreadlist_t fxas21002_DRDY[] = {{.readFrom = FXAS21002_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! Prepare the register read list to read the raw gyro data from the FXAS21002. */
const registerreadlist_t fxas21002_Output_Values[] = {
    {.readFrom = FXAS21002_OUT_X_MSB, .numBytes = FXAS21002_GYRO_DATA_SIZE}, __END_READ_DATA__};

#define RAW_ACCEL_DATA_SIZE (12)

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! Prepare the register write list to configure FXAS21002 in non-FIFO mode. */
const registerwritelist_t fxos8700_Config_Interrupt[] = {
	    /*! System and Control registers. */
	    /*! Configure the FXOS8700 to 25Hz Hybrid sampling rate. */
	    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_200_HZ, FXOS8700_CTRL_REG1_DR_MASK},
	    {FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE,
	     FXOS8700_M_CTRL_REG1_M_HMS_MASK}, /*! Enable the Hybrid Mode. */
	    {FXOS8700_M_CTRL_REG2,             /*! Enable the Data read with Hybrid Mode. */
	     FXOS8700_M_CTRL_REG2_M_AUTOINC_HYBRID_MODE | FXOS8700_M_CTRL_REG2_M_RST_CNT_DISABLE,
	     FXOS8700_M_CTRL_REG2_M_AUTOINC_MASK | FXOS8700_M_CTRL_REG2_M_RST_CNT_MASK},
	    __END_WRITE_DATA__};

/*! Command definition to read the Data Ready Status */
const registerreadlist_t FXOS8700_STATUS_READ[] = {{.readFrom = FXOS8700_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! Command definition to read the Accel + Mag Data */
const registerreadlist_t FXOS8700_ACCELMAG_READ[] = {{.readFrom = FXOS8700_OUT_X_MSB, .numBytes = 12},
                                                     __END_READ_DATA__};

/*! Command definition to read the Accel + Mag Data (in 2 TXNs). */
const registerreadlist_t FXOS8700_ACCEL_MAG_READ[] = {{.readFrom = FXOS8700_OUT_X_MSB, .numBytes = 6},
                                                      {.readFrom = FXOS8700_M_OUT_X_MSB, .numBytes = 6},
                                                      __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
volatile bool fxos8700Interrupt = false;
//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
volatile bool fxas21002Interrupt = false;

//-----------------------------------------------------------------------
// Functions

static void CalibrateMagnetometer(fxos8700_i2c_sensorhandle_t FXOS8700drv);
static void FXOS8700CQ_Accel_Calibration(fxos8700_i2c_sensorhandle_t FXOS8700drv);
static void accCalibration(fxos8700_i2c_sensorhandle_t FXOS8700drv);
//-----------------------------------------------------------------------
/*! -----------------------------------------------------------------------
 *  @brief       This is the Sensor Data Ready ISR implementation.
 *  @details     This function sets the flag which indicates if a new sample(s) is available for reading.
 *  @param[in]   pUserData This is a void pointer to the instance of the user specific data structure for the ISR.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  Yes
 *  -----------------------------------------------------------------------*/
void fxas21002_isr(void *pUserData)
{ /*! @brief Set flag to indicate Sensor has signalled data ready. */
    fxas21002Interrupt = true;
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the The main function implementation.
 *  @details     This function invokes board initializes routines, then then brings up the sensor and
 *               finally enters an endless loop to continuously read available samples.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/

void PIT_LED_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    pitIsrFlag = true;
    __DSB();
}

void accCalibration(fxos8700_i2c_sensorhandle_t FXOS8700drv)
{
	int32_t status;
	uint32_t count1;
	count1 = 0;
	fxos8700_accelmagdata_t AccelCalib;
	FLOAT64 Acc_X = 0;
	FLOAT64 Acc_Y = 0;
	FLOAT64 Acc_Z = 0;
	uint8_t data[RAW_ACCEL_DATA_SIZE];
	 uint8_t dataReady;

	do{

        status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_STATUS_READ, &dataReady);
        if (0 == (dataReady & FXOS8700_DR_STATUS_ZYXDR_MASK))
        { /* Loop, if new sample is not available. */
            continue;
        }

        /*! Read the raw sensor data from the fxos8700. */
        status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_ACCELMAG_READ, data);
        if (ARM_DRIVER_OK != status)
        {
            PRINTF("\r\n Read Failed. \r\n");

        }

        /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
        AccelCalib.accel[0] = ((int16_t)data[0] << 8) | data[1];
        AccelCalib.accel[0] /= 4;
        AccelCalib.accel[1] = ((int16_t)data[2] << 8) | data[3];
        AccelCalib.accel[1] /= 4;
        AccelCalib.accel[2] = ((int16_t)data[4] << 8) | data[5];
        AccelCalib.accel[2] /= 4;

        Acc_X = AccelCalib.accel[0];
	    /*accX = (accX/4096)*9.8;
	    rawData.accel[1] = ((int16_t)data[2] << 8) | data[3];
	    rawData.accel[1] /= 4;*/
        Acc_Y = AccelCalib.accel[1];
	    /*accY = (accX/4096)*9.8;
	    rawData.accel[2] = ((int16_t)data[4] << 8) | data[5];
	    rawData.accel[2] /= 4;*/
        Acc_Z = AccelCalib.accel[2];
	    /*accZ = (accZ/4096)*9.8;*/


	    sstatex = sstatex + Acc_X;
	    sstatey = sstatey + Acc_Y;
	    sstatez = sstatez + Acc_Z;

        count1++;
	}while (count1 != 0x0800);


	sstatex = sstatex / 2048;
	sstatey = sstatey / 2048;
	sstatez = sstatez / 2048;

}

void MagneticFieldOffset(fxos8700_i2c_sensorhandle_t FXOS8700drv)
{

	    uint16_t i = 0;
	    int32_t status;
	    uint8_t dataReady;


	    fxos8700_accelmagdata_t Mag_16_bit;
	    uint8_t AccelMagData[RAW_ACCEL_DATA_SIZE];
	    while(i < 3000)
	    {
	    	status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_STATUS_READ, &dataReady);
	    	if (dataReady)
	    	{
	    		dataReady = 0;

	    		status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_ACCELMAG_READ, AccelMagData);

	    		Mag_16_bit.mag[0] = ((int16_t)AccelMagData[6]<<8 | AccelMagData[7]);        // Compute 16-bit X-axis magnetic output value
	    		Mag_16_bit.mag[1] = ((int16_t)AccelMagData[8]<<8 | AccelMagData[9]);        // Compute 16-bit Y-axis magnetic output value
	    		Mag_16_bit.mag[2] = ((int16_t)AccelMagData[10]<<8 | AccelMagData[11]);        // Compute 16-bit Z-axis magnetic output value
	    		MagneticFieldX += (Mag_16_bit.mag[0]-offset_x)*scale_x;
	    		MagneticFieldY += (Mag_16_bit.mag[1]-offset_y)*scale_y;
	    		MagneticFieldZ += (Mag_16_bit.mag[2]-offset_z)*scale_z;
	            i++;
	    	}
	    }
	    MagneticFieldX /= i;
	    MagneticFieldY /= i;
	    MagneticFieldZ /= i;
}

void CalibrateMagnetometer(fxos8700_i2c_sensorhandle_t FXOS8700drv)
{
    int16_t Xout_Mag_16_bit_avg, Yout_Mag_16_bit_avg, Zout_Mag_16_bit_avg;
    int16_t Xout_Mag_16_bit_max, Yout_Mag_16_bit_max, Zout_Mag_16_bit_max;
    int16_t Xout_Mag_16_bit_min, Yout_Mag_16_bit_min, Zout_Mag_16_bit_min;
    uint16_t i = 0;
    int32_t status;
    uint8_t dataReady;


    fxos8700_accelmagdata_t Mag_16_bit;
    uint8_t AccelMagData[RAW_ACCEL_DATA_SIZE];
    while(i < 3000)
    {
    	status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_STATUS_READ, &dataReady);
    	if (dataReady)
    	{
    		dataReady = 0;

    		status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_ACCELMAG_READ, AccelMagData);

    		Mag_16_bit.mag[0] = ((int16_t)AccelMagData[6]<<8 | AccelMagData[7]);        // Compute 16-bit X-axis magnetic output value
    		Mag_16_bit.mag[1] = ((int16_t)AccelMagData[8]<<8 | AccelMagData[9]);        // Compute 16-bit Y-axis magnetic output value
    		Mag_16_bit.mag[2] = ((int16_t)AccelMagData[10]<<8 | AccelMagData[11]);        // Compute 16-bit Z-axis magnetic output value

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
             i++;
    	}
    }

    Xout_Mag_16_bit_avg = (Xout_Mag_16_bit_max + Xout_Mag_16_bit_min) / 2;            // X-axis hard-iron offset
    Yout_Mag_16_bit_avg = (Yout_Mag_16_bit_max + Yout_Mag_16_bit_min) / 2;            // Y-axis hard-iron offset
    Zout_Mag_16_bit_avg = (Zout_Mag_16_bit_max + Zout_Mag_16_bit_min) / 2;            // Z-axis hard-iron offset

    offset_x =Xout_Mag_16_bit_avg;
    offset_y =Yout_Mag_16_bit_avg;
    offset_z =Zout_Mag_16_bit_avg;

    Xout_Mag_16_bit_avg = (Xout_Mag_16_bit_max - Xout_Mag_16_bit_min) / 2;            // X-axis hard-iron offset
    Yout_Mag_16_bit_avg = (Yout_Mag_16_bit_max - Yout_Mag_16_bit_min) / 2;            // Y-axis hard-iron offset
    Zout_Mag_16_bit_avg = (Zout_Mag_16_bit_max - Zout_Mag_16_bit_min) / 2;            // Z-axis hard-iron offset

    float avg_delta;
    avg_delta = (Xout_Mag_16_bit_avg+Yout_Mag_16_bit_avg+Zout_Mag_16_bit_avg)/3;
    scale_x=avg_delta/(float)Xout_Mag_16_bit_avg;
    scale_y=avg_delta/(float)Yout_Mag_16_bit_avg;
    scale_z=avg_delta/(float)Zout_Mag_16_bit_avg;

}

void FXOS8700CQ_Accel_Calibration(fxos8700_i2c_sensorhandle_t FXOS8700drv)
{
    float Xout_Accel_14_bit, Yout_Accel_14_bit, Zout_Accel_14_bit;
    float Xmax_Accel_14_bit, Ymax_Accel_14_bit, Zmax_Accel_14_bit;
    float Xmin_Accel_14_bit, Ymin_Accel_14_bit, Zmin_Accel_14_bit;
    uint8_t AccelMagData[RAW_ACCEL_DATA_SIZE];
    int32_t status;
    uint16_t i =0;
    uint8_t dataReady;
    while(i < 50)
    {
    	status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_STATUS_READ, &dataReady);
    	if (dataReady)
    	{
    		dataReady = 0;
	status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_ACCELMAG_READ, AccelMagData);

		Xout_Accel_14_bit = ((int16_t)AccelMagData[0] << 8) | AccelMagData[1];

		Yout_Accel_14_bit = ((int16_t)AccelMagData[2] << 8) | AccelMagData[3];

		Zout_Accel_14_bit = ((int16_t)AccelMagData[4] << 8) | AccelMagData[5];
		if(i==0)
		{
			Xmax_Accel_14_bit=Xout_Accel_14_bit;
			Ymax_Accel_14_bit=Yout_Accel_14_bit;
			Zmax_Accel_14_bit=Zout_Accel_14_bit;
			Xmin_Accel_14_bit=Xout_Accel_14_bit;
			Ymin_Accel_14_bit=Yout_Accel_14_bit;
			Zmin_Accel_14_bit=Zout_Accel_14_bit;
		}

		if(Xmax_Accel_14_bit<Xout_Accel_14_bit && Xout_Accel_14_bit!=0)
		{
			Xmax_Accel_14_bit=Xout_Accel_14_bit;
		}
		if(Ymax_Accel_14_bit<Yout_Accel_14_bit && Yout_Accel_14_bit!=0)
		{
			Ymax_Accel_14_bit=Yout_Accel_14_bit;
		}
		if(Zmax_Accel_14_bit<Zout_Accel_14_bit && Zout_Accel_14_bit!=0)
		{
			Zmax_Accel_14_bit=Zout_Accel_14_bit;
		}
		if(Xmin_Accel_14_bit>Xout_Accel_14_bit && Xout_Accel_14_bit!=0)
		{
			Xmin_Accel_14_bit=Xout_Accel_14_bit;
		}
		if(Ymin_Accel_14_bit>Yout_Accel_14_bit && Yout_Accel_14_bit!=0)
		{
			Ymin_Accel_14_bit=Yout_Accel_14_bit;
		}
		if(Zmin_Accel_14_bit>Zout_Accel_14_bit && Zout_Accel_14_bit!=0)
		{
			Zmin_Accel_14_bit=Zout_Accel_14_bit;
		}
		i++;
    	}

    }

    X_Accel_offset = 0.5*(Xmax_Accel_14_bit-Xmin_Accel_14_bit);  // Compute X-axis offset correction value
    Y_Accel_offset = 0.5*(Ymax_Accel_14_bit-Ymin_Accel_14_bit);  // Compute X-axis offset correction value
    Z_Accel_offset = 0.5*(Zmax_Accel_14_bit-Zmin_Accel_14_bit);  // Compute X-axis offset correction value

    GainX = 0.5*((Xmax_Accel_14_bit-Xmin_Accel_14_bit)/4096);
    GainY = 0.5*((Ymax_Accel_14_bit-Ymin_Accel_14_bit)/4096);
    GainZ = 0.5*((Zmax_Accel_14_bit-Zmin_Accel_14_bit)/4096);
}


int main(void)
{
	uart_config_t config;
	pit_config_t pitConfig;
    int32_t status;
    //uint32_t dt;

    uint8_t Count=0;

    uint8_t dataReady;
    uint8_t data2[RAW_ACCEL_DATA_SIZE];
    uint8_t data[FXAS21002_GYRO_DATA_SIZE];

    fxos8700_accelmagdata_t rawData2;
    fxas21002_gyrodata_t rawData;
    FLOAT64 X=0;
    FLOAT64 Y=0;
    FLOAT64 Z=0;
    float gyroX, gyroY, gyroZ;

    float magx;
    float magy;
    float magz;

    ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!
    fxos8700_i2c_sensorhandle_t FXOS8700drv;
    fxas21002_i2c_sensorhandle_t FXAS21002drv;
    GENERIC_DRIVER_GPIO *gpioDriver = &Driver_GPIO_KSDK;



    /*! Initialize the MCU hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\n ISSDK FXAS21002 sensor driver example demonstration with interrupt mode.\r\n");

    /*! Initialize INT1_FXAS21002 pin used by FRDM board */
    gpioDriver->pin_init(&FXAS21002_INT1, GPIO_DIRECTION_IN, NULL, &fxas21002_isr, NULL);

    /*! Initialize INT1_FXOS8700 pin used by FRDM board */
    //gpioDriver->pin_init(&FXOS8700_INT2, GPIO_DIRECTION_IN, NULL, &fxos8700_isr_callback, NULL);

    /*! Initialize RGB LED pin used by FRDM board */
    gpioDriver->pin_init(&GREEN_LED, GPIO_DIRECTION_OUT, NULL, NULL, NULL);

    GPIO_pinControlRegisterType pinControlRegisterPORTC = GPIO_MUX1;
    GPIO_clockGating(GPIO_C);
    GPIO_pinControlRegister(GPIO_C,BIT3,&pinControlRegisterPORTC);
    GPIO_dataDirectionPIN(GPIO_C,GPIO_OUTPUT,BIT3);
    GPIO_tooglePIN(GPIO_C,BIT3);

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Initialization Failed\r\n");
        return -1;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Power Mode setting Failed\r\n");
        return -1;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Control Mode setting Failed\r\n");
        return -1;
    }


    /*! Initialize the FXOS8700 sensor driver. */
    status = FXOS8700_I2C_Initialize(&FXOS8700drv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXOS8700_I2C_ADDR,
                                     FXOS8700_WHO_AM_I_PROD_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n Sensor Initialization Failed\r\n");
        return -1;
    }
    PRINTF("\r\n Successfully Initiliazed Sensor\r\n");
    /*! Initialize the FXAS21002 sensor driver. */
    status = FXAS21002_I2C_Initialize(&FXAS21002drv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXAS21002_I2C_ADDR,
                                      FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);


    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n Sensor Initialization Failed\r\n");
        return -1;
    }
    PRINTF("\r\n Successfully Initiliazed Sensor\r\n");

    /*!  Set the task to be executed while waiting for I2C transactions to complete. */
    FXAS21002_I2C_SetIdleTask(&FXAS21002drv, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);

    /*! Configure the FXAS21002 sensor driver. */
    status = FXAS21002_I2C_Configure(&FXAS21002drv, fxas21002_Config_Isr);

    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n FXAS21002 Sensor Configuration Failed, Err = %d\r\n", status);
        return -1;
    }

    PRINTF("\r\n Successfully Applied FXAS21002 Sensor Configuration\r\n");
    /*!  Set the task to be executed while waiting for I2C transactions to complete. */
    FXOS8700_I2C_SetIdleTask(&FXOS8700drv, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);

    /*! Configure the fxos8700 sensor driver. */
    status = FXOS8700_I2C_Configure(&FXOS8700drv, fxos8700_Config_Interrupt);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n FXOS8700 Sensor Configuration Failed, Err = %d\r\n", status);
        return -1;
    }
    PRINTF("\r\n Successfully Applied FXOS8700 Sensor Configuration\r\n");
	#if CALIBRATE_MAGNETOMETER
    	CalibrateMagnetometer( FXOS8700drv);
    	PRINTF("scale_x	%f	scale_y	%f	scale_z	%f offset_x	%f	offset_y	%f	offset_z	%f\n\r", scale_x,  scale_y,  scale_z, offset_x, offset_y, offset_z);
    	return -1;
	#endif


    /*UART INITIALIZATION*/
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    //UART_WriteBlocking(DEMO_UART, txbuff, sizeof(txbuff) - 1);

    /*PIT INITIALIZATION*/
    PIT_GetDefaultConfig(&pitConfig);

    /* Init pit module */
    PIT_Init(PIT, &pitConfig);

    /* Set timer period for channel 0 */
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(5000, PIT_SOURCE_CLOCK));

    /* Enable timer interrupts for channel 0 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    //FXOS8700CQ_Accel_Calibration(FXOS8700drv);
    /* Enable at the NVIC */
    EnableIRQ(PIT_IRQ_ID);
    PIT_StartTimer(PIT, kPIT_Chnl_0);

    //accCalibration(FXOS8700drv);
    MagneticFieldOffset(FXOS8700drv);
    for (;;) /* Forever loop */
    {        /* In ISR Mode we do not need to check Data Ready Register.
              * The receipt of interrupt will indicate data is ready. */
        		status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_STATUS_READ, &dataReady);

                /*! Read the raw sensor data from the fxos8700. */
                status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_ACCELMAG_READ, data2);

        /*! Read the raw sensor data from the FXAS21002. */
        status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_Output_Values, data);

        if(pitIsrFlag)
        {
        	GPIO_tooglePIN(GPIO_C,BIT3);
        	pitIsrFlag = false;
            rawData2.accel[0] = ((int16_t)data2[0] << 8) | data2[1];
            rawData2.accel[0] /= 4;
            X = rawData2.accel[0] - sstatex;
            X = (X)/4096;

            rawData2.accel[1] = ((int16_t)data2[2] << 8) | data2[3];
            rawData2.accel[1] /= 4;
            Y = rawData2.accel[1] - sstatey;
            Y = (Y)/4096;
            rawData2.accel[2] = ((int16_t)data2[4] << 8) | data2[5];
            rawData2.accel[2] /= 4;
            Z = rawData2.accel[2] - sstatez;
            Z = (Z)/4096;

			#if MAGNETOMETER_CALIBON
            	rawData2.mag[0] = ((((int16_t)data2[6] << 8) | data2[7]));
                magx =(rawData2.mag[0]);

                rawData2.mag[1] = ((((int16_t)data2[8] << 8) | data2[9]));
                magy =(rawData2.mag[1]);

                rawData2.mag[2] = ((((int16_t)data2[10] << 8) | data2[11]));
                magz =(rawData2.mag[2]);
			#else
            	rawData2.mag[0] = ((((int16_t)data2[6] << 8) | data2[7]));
                magx =(rawData2.mag[0]-offset_x)*scale_x;

                rawData2.mag[1] = ((((int16_t)data2[8] << 8) | data2[9]));
                magy =(rawData2.mag[1]-offset_y)*scale_y;

                rawData2.mag[2] = ((((int16_t)data2[10] << 8) | data2[11]));
                magz =(rawData2.mag[2]-offset_z)*scale_z;
			#endif

            /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
            rawData.gyro[0] = ((int16_t)data[0] << 8) | data[1];
            rawData.gyro[1] = ((int16_t)data[2] << 8) | data[3];
            rawData.gyro[2] = ((int16_t)data[4] << 8) | data[5];

            gyroX = (float)rawData.gyro[0]/131;
            gyroY = (float)rawData.gyro[1]/131;
            gyroZ = (float)rawData.gyro[2]/131;
			#if MAGNETOMETER_CALIB
            	PRINTF("Xmag	%f	Ymag	%f	Zmag	%f\n\r", magx*100,  magy*100,  magz*100);
			#elif MAGNETOMETER_RAY_TEST
            	float Vector = sqrtf(pow(((magx-MagneticFieldX)/100),2) + pow(((magy-MagneticFieldY)/100), 2) + pow(((magz-MagneticFieldZ)/100),2) );

            	PRINTF("Vector	%f	X:	%f	Y:	%f	Z:	%f	\r", Vector,((magx-MagneticFieldX)/100),((magy-MagneticFieldY)/100),((magz-MagneticFieldZ)/100));

            	//PRINTF("X = %f \n\r", posY);
            	/*
            	GPIO_tooglePIN(GPIO_C,BIT3);
            	MadgwickAHRSupdate((gyroZ*3.1416/180), (gyroY*3.1416/180), (gyroX*3.1416/180), Z, Y, X, magz, magy,magx);
            	GPIO_tooglePIN(GPIO_C,BIT3);*/
			#else
            	PRINTF("Xmag	%f	Ymag	%f	Zmag	%f \r\n", (magx-MagneticFieldX)/100,  (magy-MagneticFieldY)/100,  (magz-MagneticFieldZ)/100);
            	MadgwickAHRSupdate((gyroZ*3.1416/180), (gyroY*3.1416/180), (gyroX*3.1416/180), Z, Y, X, magz, magy,magx);
            #endif


         }
    }
}

