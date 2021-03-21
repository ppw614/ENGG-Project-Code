/******************************************************************************
 * Name:    		Main.c
 * Description: The main program file
 * Version: 		Version 1.00
 * Authors: 		Gunjeet Dhaliwal | Mckenzie Busenius
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 *----------------------------------------------------------------------------
 * History:
 *          V1.00 Initial Version
 *
 *****************************************************************************/

#include "stm32f10x.h"
#include "clocks.h"
#include "motion_Sensor.h"
#include "LCD.h"

int main(void){
		clockInit();
		subclocks();
		GPIOA->ODR = 0x1E00;
		LCD_IO_INIT();
		while(1)
		{
			GPIOA->ODR = 0x1E00;
			commandToLCD(LCD_CLR);
			while((motion_Test() == 1))
			{
				dataToLCD('M');
				dataToLCD('O');
				dataToLCD('T');
				dataToLCD('I');
				dataToLCD('O');
				dataToLCD('N');
				commandToLCD(LCD_LN2);	
				dataToLCD('D');
				dataToLCD('E');
				dataToLCD('T');
				dataToLCD('E');
				dataToLCD('C');
				dataToLCD('T');
				dataToLCD('E');
				dataToLCD('D');
				commandToLCD(LCD_LN1);
				GPIOA->ODR = 0x1E00;
				GPIOA->ODR &= ~GPIO_ODR_ODR9 & ~GPIO_ODR_ODR10;
			}		
			commandToLCD(LCD_CLR);
			while((motion_Test() == 0))
			{
				GPIOA->ODR = 0x1E00;
				dataToLCD('N');
				dataToLCD('O');
				dataToLCD(' ');
				dataToLCD('M');
				dataToLCD('O');
				dataToLCD('T');
				dataToLCD('I');
				dataToLCD('O');
				dataToLCD('N');
				commandToLCD(LCD_LN2);	
				dataToLCD('D');
				dataToLCD('E');
				dataToLCD('T');
				dataToLCD('E');
				dataToLCD('C');
				dataToLCD('T');
				dataToLCD('E');
				dataToLCD('D');
				commandToLCD(LCD_LN1);
			}		
		}
}	