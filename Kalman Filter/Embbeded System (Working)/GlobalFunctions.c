/**
	\file
		GlobalFunctions.c
	\brief
		This is a source file to use the GlobalFunctions.
		i. e. Delays by counter and NULL function
	\author José Luis Pizano
	\date	09/03/2018
 */


#include "GlobalFunctions.h"


/**Makes a desired delay in time*/
void delay(uint16 delay)
{
	volatile int counter, counter2;

	/**Performs delay through loops of for*/
	for(counter2=16; counter2 > 0; counter2--)
	{
		for(counter=delay; counter > 0; counter--);

	}
}

/**NULL function*/
void NOP()
{

}
