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

