/******************************************************************************
* Name: 				infrared_Sensor.c
* Description: 	Detects any nearby obstacles
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
#include "infrared_sensor.h"
#include "LCD.h"

void start_Motion_Sensor(void){
	obstacle_Test();
}

int obstacle_Test(void){
	if((GPIOB->IDR & GPIO_IDR_IDR4) != GPIO_IDR_IDR4)	
		return 0;
	return 1;
}

