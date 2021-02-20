/******************************************************************************
 * Name:      		IR.c
 * Description: 	Implementation of IR sensor, Front, Back, Left and Right.
 * Version: 		  Version 1.00
 * Authors: 		  Gunjeet Dhaliwal | Mckenzie Busenius
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 *
 *----------------------------------------------------------------------------
 * History:
 *          V1.00 Initial Version
 *          
 *****************************************************************************/
#include "stm32f10x.h"
#include "IR.h"
#include "LCD.h"
#include "Motor.h"

void start_IR(void){
	IR_Detect_Front();
	IR_Detect_Right();
	IR_Detect_Left();
}

int IR_Detect_Front(void){
	if((GPIOB->IDR & GPIO_IDR_IDR6) != GPIO_IDR_IDR6)	
		return 0;
	return 1;
}

int IR_Detect_Right(void){
	if((GPIOB->IDR & GPIO_IDR_IDR7) != GPIO_IDR_IDR7)	
		return 0;
	return 1;
}

int IR_Detect_Left(void){
	if((GPIOB->IDR & GPIO_IDR_IDR4) != GPIO_IDR_IDR4)	
		return 0;
	return 1;
}
