/**
	\file
		GlobalFunctions.h
	\brief
		This is a header file to use the GlobalFunctions.
		i. e. Delays by counter and NULL function
	\author José Luis Pizano
	\date	09/03/2018
 */

#ifndef GLOBALFUNCTIONS_H_
#define GLOBALFUNCTIONS_H_

#include "DataTypeDefinitions.h"


/*!
 	 \brief	 This function makes a delay in time.

 	 \param[in]  delay Port to clear interrupts.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void delay(uint16 delay);


/*!
 	 \brief	 NULL function

 	 \return void
 */
void NOP();


#endif /* GLOBALFUNCTIONS_H_ */
