/******************************************************************************
* Name: 				motion_Sensor.c
* Description: 	Header File
* Version: 			Version 1.00
* Authors: 			Gunjeet Dhaliwal
*
* This software is supplied "AS IS" without warranties of any kind.
*
*
*----------------------------------------------------------------------------
* History:			V1.00 Initial Version
*
*****************************************************************************/

#include "stm32f10x.h"
#include "motion_sensor.h"
#include "LCD.h"

void start_IR(void){
	motion_Test();
}

int motion_Test(void){
	if((GPIOB->IDR & GPIO_IDR_IDR4) != GPIO_IDR_IDR4)	
		return 0;
	return 1;
}

