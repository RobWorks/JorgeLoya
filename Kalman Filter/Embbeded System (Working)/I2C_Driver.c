/*
 * I2C_Driver.c
 *
 *  Created on: Apr 6, 2018
 *      Author: Jorge Loya
 */
#include "MK64F12.h"
#include "I2C_Driver.h"
#include "GPIO.h"
#define MUL 0x1
#define LOAD_I2Cx 0x14
uint8 Flag;
uint8 Busy;
uint8 Data;
uint8 ReadData;

void I2C_init(I2C_ChannelType channel, uint32 systemClock, uint8 baudRate)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
						I2C0->F |= LOAD_I2Cx;
						I2C0->C1 |= I2C_C1_IICEN_MASK;
						break;
					case I2C_1:/** GPIO B is selected*/
						SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
						I2C1->F |= LOAD_I2Cx;
						I2C1->C1 |= I2C_C1_IICEN_MASK;
						break;
					case I2C_2:/** GPIO C is selected*/
						SIM->SCGC1 |= SIM_SCGC1_I2C2_MASK;
						I2C2->F |= LOAD_I2Cx;
						I2C2->C1 |= I2C_C1_IICEN_MASK;
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
	GPIO_pinControlRegisterType MUXAlt2 = GPIO_MUX2;
	GPIO_clockGating(GPIO_B);
	GPIO_pinControlRegister(GPIO_B,BIT2,&MUXAlt2);
	GPIO_pinControlRegister(GPIO_B,BIT3,&MUXAlt2);

	return;
}

uint8 I2C_busy(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						Busy = (I2C0->S & I2C_S_BUSY_MASK) << I2C_S_BUSY_SHIFT;
						break;
					case I2C_1:/** GPIO B is selected*/
						Busy = (I2C1->S & I2C_S_BUSY_MASK) << I2C_S_BUSY_SHIFT;
						break;
					case I2C_2:/** GPIO C is selected*/
						Busy = (I2C2->S & I2C_S_BUSY_MASK) << I2C_S_BUSY_SHIFT;
						break;
					default:/**If doesn't exist the option*/
						return 0;
						break;
					}
	return Busy;
}

void I2C_stop(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						I2C0->C1 &= ~I2C_C1_MST_MASK;
						I2C0->C1 &= ~I2C_C1_TX_MASK;
						break;
					case I2C_1:/** GPIO B is selected*/
						I2C1->C1 &= ~I2C_C1_MST_MASK;
						I2C1->C1 &= ~I2C_C1_TX_MASK;
						break;
					case I2C_2:/** GPIO C is selected*/
						I2C2->C1 &= ~I2C_C1_MST_MASK;
						I2C2->C1 &= ~I2C_C1_TX_MASK;
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
}

void I2C_MST_OrSLV_Mode(uint8 masterOrSlave, I2C_ChannelType channel)
{
	switch(channel)
	{
	case I2C_0:/** GPIO A is selected*/
		if (masterOrSlave)
		{
			I2C0->C1 |= I2C_C1_MST_MASK;
		}else
		{
			I2C0->C1 &= ~I2C_C1_MST_MASK;
		}
		break;
	case I2C_1:/** GPIO B is selected*/
		if (masterOrSlave)
		{
			I2C1->C1 |= I2C_C1_MST_MASK;
		}else
		{
			I2C1->C1 &= ~I2C_C1_MST_MASK;
		}
		break;
	case I2C_2:/** GPIO C is selected*/
		if (masterOrSlave)
		{
			I2C2->C1 |= I2C_C1_MST_MASK;
		}else
		{
			I2C2->C1 &= ~I2C_C1_MST_MASK;
		}
		break;
	default:/**If doesn't exist the option*/
		return;
		break;
	}
}

void I2C_TX_RX_Mode(uint8 TxOrRx, I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						if(TxOrRx)
						{
							I2C0->C1 |= I2C_C1_TX_MASK;
						}else
						{
							I2C0->C1 &= ~I2C_C1_TX_MASK;
						}
						break;
					case I2C_1:/** GPIO B is selected*/
						if(TxOrRx)
						{
							I2C1->C1 |= I2C_C1_TX_MASK;
						}else
						{
							I2C1->C1 &= ~I2C_C1_TX_MASK;
						}
						break;
					case I2C_2:/** GPIO C is selected*/
						if(TxOrRx)
						{
							I2C2->C1 |= I2C_C1_TX_MASK;
						}else
						{
							I2C2->C1 &= ~I2C_C1_TX_MASK;
						}
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
}

void I2C_start(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						I2C0->C1 |= I2C_C1_TX_MASK;
						I2C0->C1 |= I2C_C1_MST_MASK;
						break;
					case I2C_1:/** GPIO B is selected*/
						I2C1->C1 |= I2C_C1_MST_MASK;
						I2C1->C1 |= I2C_C1_TX_MASK;
						break;
					case I2C_2:/** GPIO C is selected*/
						I2C2->C1 |= I2C_C1_MST_MASK;
						I2C2->C1 |= I2C_C1_TX_MASK;
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
}

void I2C_NACK(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						I2C0->C1 |= I2C_C1_TXAK_MASK;
						break;
					case I2C_1:/** GPIO B is selected*/
						I2C1->C1 |= I2C_C1_TXAK_MASK;
						break;
					case I2C_2:/** GPIO C is selected*/
						I2C2->C1 |= I2C_C1_TXAK_MASK;
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
}

void I2C_repeted_Start(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						I2C0->C1 |= I2C_C1_RSTA_MASK;
						break;
					case I2C_1:/** GPIO B is selected*/
						I2C1->C1 |= I2C_C1_RSTA_MASK;
						break;
					case I2C_2:/** GPIO C is selected*/
						I2C2->C1 |= I2C_C1_RSTA_MASK;
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
}

void I2C_write_Byte(uint8 Data, I2C_ChannelType channel){
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						I2C0->D = Data;
						break;
					case I2C_1:/** GPIO B is selected*/
						I2C1->D = Data;
						break;
					case I2C_2:/** GPIO C is selected*/
						I2C2->D = Data;
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
}

uint8  I2C_read_Byte(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						ReadData = I2C0->D;
						return ReadData;
						break;
					case I2C_1:/** GPIO B is selected*/
						ReadData = I2C1->D;
						return ReadData;
						break;
					case I2C_2:/** GPIO C is selected*/
						ReadData = I2C2->D;
						return ReadData;
						break;
					default:/**If doesn't exist the option*/
						return 0;
						break;
					}
}

uint8 I2C_get_ACK(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						//Ack = (I2C0->S & I2C_S_RXAK_MASK);
						return (I2C0->S & I2C_S_RXAK_MASK);
						break;
					case I2C_1:/** GPIO B is selected*/
						return (I2C1->S & I2C_S_RXAK_MASK);
					case I2C_2:/** GPIO C is selected*/
						return (I2C2->S & I2C_S_RXAK_MASK);
					default:/**If doesn't exist the option*/
						return 0;
						break;
					}
}

void I2C_wait(I2C_ChannelType channel)
{
	switch(channel)
					{
					case I2C_0:/** GPIO A is selected*/
						while(FALSE == (I2C0->S & I2C_S_IICIF_MASK));
						I2C0->S |= I2C_S_IICIF_MASK;
						break;
					case I2C_1:/** GPIO B is selected*/
						while(FALSE == (I2C1->S & I2C_S_IICIF_MASK));
						I2C1->S |= I2C_S_IICIF_MASK;
						break;
					case I2C_2:/** GPIO C is selected*/
						while(FALSE == (I2C2->S & I2C_S_IICIF_MASK));
						I2C2->S |= I2C_S_IICIF_MASK;
						break;
					default:/**If doesn't exist the option*/
						return;
						break;
					}
}
