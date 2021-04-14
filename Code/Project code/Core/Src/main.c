/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @Authors         : Wangwe Paul |Gunjeet Dhaliwal | Gurinder Brar | Vitalii Andriievskyi
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *Purpose: code used on semi-Autonomous UVC disinfectant robot
 *Works like a state machine
 *state 1 idle state
 *state 2 check for motion
 *state 3 move robot foward
 *state 4 big right turn
 *state 5 minor left turn
 *state 6 minor right turn
 *state 7 big left turn
 *state 8 move robot in reverse
 **Port mapping
 *Motion Sensors(F,R,L,B)
 *Inputs : Front - PB10
 	   Left  - PB15
	   Right - PA10
	   Rear  - PB3
 *Ultrasonic Sensors(F,R,L,B)
 *Triggers/Outputs: Front - PC1
 		    Right - PC0
		    Left  - PC2
		    Rear  - PC3
 *Echo/Inputs:      Front - PB5
 		    Right - PB4
		    Left  - PB14
		    Rear  - PB13
 *Infrared sensors(FR, FL, BR, Bl)
 *Inputs: Front-Right - PA0
 	  Front-Left  - PA1
	  Rear-Right  - PA4
	  Rear-Left   - PB0
 *Motors
 *motors are controlled by another microcontorller
 *so the following ports are mutually exclusive
 *foward movement   - PC11
 *turing right      - PC10
 *turning left      - PB12
 *backward movement - PB11
 *stop              - PC12
 *pin used to sense if cable is still there
 *winch pin         - PC4
 *pin used to indicate end operation
 *end operation     - PC9 
 *Light Control
 *PA9 high light is off
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Motors_Fw(void);
void Motors_Rv(void);
void Turn_R(void);
void Turn_L(void);
void Motors_Stp(void);
void Delay(uint32_t count);

uint16_t ultraSonicF(void);
uint16_t ultraSonicR(void);
uint16_t ultraSonicL(void);
uint16_t ultraSonicB(void);

int state = 1; //shows the state the robot is in
// 1 default state robot waits 5 minutes before turning on light
// 2 Robot can move
// 3 Robot stops to check surroundings
// 4 Robot turns because of obstacle ahead
// 5 Robot turns because of right ir sensor
// 6 Robot turns because of left ir sensor
// 7 Robot reversing

const uint32_t motorDelay = 480;
//used to control the delay of the motor
//higher -> slower rotation and lower -> faster rotation

const uint16_t defVal = 4000;
//used as default value for distance measurement using ultrasonic
//distance value will never be 4000

const uint16_t obstDistance = 16;
//maximum distance from robot to obstacle when moving ahead
const uint16_t obstDistance2 = 10;
//maximum distance from robot to obstacle when turning
const uint16_t turnDist = 150;
//maximum distance from robot to obstacle when turning

const uint16_t idleTime = 250;
//time the robot spend idle
const uint16_t moveTime = 10;
//time the robot spend moving foward or backward
const uint16_t turnTime=5;
//time the robot spends doing big turns
const uint16_t turnTime2=2;
//time the robot spends doing smaller turns
const uint16_t checkTime=6;
//time the robot spends checking for motion


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, 1);
	int Secs = 0;
	// where 15 is approximately to 10 seconds
	int timcounter = 0;
	uint16_t frontDistance = 0;
	uint16_t rightDistance = 0;
	uint16_t leftDistance = 0;
	uint16_t backDistance = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (state == 1) {
			//TURN OFF UVC LIGHT
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			Motors_Stp();
			Secs = 0; //reset counter used for timing
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <= idleTime) {
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
			//output from second microcontroller which is high when robot has gone foward and come back
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)) {
				state = 1;
			} else {
				state = 2;
			}

		}
		if (state == 2) {

			Secs = 0; //reset counter used for timing
			Motors_Stp();
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <=checkTime) {
				if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10))
						|| (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
						|| (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
						|| (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))) {
					state = 1;
					Secs = 8;
				}
				//checks if output from second microcontroller which goes high if all the extension cord has been uncound
				else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)) {
					state = 8;
				} else {
					state = 3;
				}
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
		}
		if (state == 3) {
			//TURN ON UVC LIGHT
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			Secs = 0; //reset counter used for timing
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <= moveTime) {
				Motors_Fw();
				if (Secs == 0 || Secs == 3 || Secs == 6) {
					Motors_Stp();
					frontDistance = ultraSonicF();
					rightDistance = ultraSonicF();
					leftDistance = ultraSonicF();
					if (frontDistance < obstDistance
							&& rightDistance > obstDistance2
							&& leftDistance > obstDistance2) {
						state = 4;
						Secs = moveTime+1;
					}
					if (frontDistance < obstDistance
							&& rightDistance > obstDistance2
							&& leftDistance < obstDistance2) {
						state = 4;
						Secs = moveTime+1;
					}
					if (frontDistance < obstDistance
							&& rightDistance < obstDistance2
							&& leftDistance > obstDistance2) {
						state = 7;
						Secs = moveTime+1;
					}

				}
				//front right IR sensor
				else if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
					state = 5;
					Secs = moveTime+1;
				}
				// front left IR sensor
				else if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
					state = 6;
					Secs = moveTime+1;
				}
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
			if (state == 3) {
				state = 2;
			}

		}

		if (state == 4) {
			Motors_Stp();
			HAL_Delay(2000);
			Secs = 0; //reset counter used for timing
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <= turnTime) {
				Turn_R();
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
			Motors_Stp();
			state = 2;
		}
		if (state == 5) {
			Motors_Stp();
			HAL_Delay(2000);
			Secs = 0; //reset counter used for timing
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <= turnTime2) {
				Turn_L();
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
			Motors_Stp();
			state = 2;
		}
		if (state == 6) {
			Motors_Stp();
			HAL_Delay(2000);
			Secs = 0; //reset counter used for timing
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <= TurnTime2) {
				Turn_R();
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
			Motors_Stp();
			state = 2;
		}
		if (state == 7) {
			Motors_Stp();
			HAL_Delay(2000);
			Secs = 0; //reset counter used for timing
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <=turnTime) {
				Turn_L();
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
			Motors_Stp();
			state = 2;
		}
		if (state == 8) {
			//TURN ON UVC LIGHT
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			Secs = 0; //reset counter used for timing
			__HAL_TIM_SET_COUNTER(&htim4, 0); //reset timer
			while (Secs <= moveTime) {
				Motors_Rv();
				if (Secs == 0 || Secs == 3 || Secs == 6) {
					Motors_Stp();
					backDistance = ultraSonicF();
					rightDistance = ultraSonicF();
					leftDistance = ultraSonicF();
					if (backDistance < obstDistance
							&& rightDistance > obstDistance2
							&& leftDistance > obstDistance2) {
						state = 4;
						Secs = moveTime+1;
					}
					if (backDistance < obstDistance
							&& rightDistance > obstDistance2
							&& leftDistance < obstDistance2) {
						state = 7;
						Secs = moveTime+1;
					}
					if (backDistance < obstDistance
							&& rightDistance < obstDistance2
							&& leftDistance > obstDistance2) {
						state = 4;
						Secs = moveTime+1;
					}
				}
				//back left IR sensor
				else if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
					state = 5;
					Secs = moveTime+1;
				}
				//back right IR sensor
				else if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
					state = 6;
					Secs = moveTime+1;
				}
				timcounter = __HAL_TIM_GET_COUNTER(&htim4);
				if (timcounter > 105) {
					Secs++;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
				}
			}
			if (state == 8) {
				state = 2;
			}

		}

	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 49999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 4000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_10
					| GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC0 PC1 PC2 PC3
	 PC10 PC11 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
			| GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 PA10 */
	GPIO_InitStruct.Pin =
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PC4 PC6 PC8 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB10 PB13 PB15
	 PB3 PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_15
			| GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB11 PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Delay(uint32_t count) {
	for (int i = 0; i < count; i++) {

	}
}
void Motors_Fw(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}
void Motors_Rv(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}
void Turn_L(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}
void Turn_R(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}
void Motors_Stp(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
}
uint16_t ultraSonicR() {
	uint8_t status = 0;
	uint32_t distance = defVal;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(0.001);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(0.01);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) {
		status = 1;
		distance = 0;

		while (status == 1) {
			Delay(10);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) {
				distance++;
			} else {
				status = 0;
			}
		}
	}

	return (distance);
}
uint16_t ultraSonicF() {
	uint8_t status = 0;
	uint32_t distance = defVal;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(0.001);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(0.01);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) {
		status = 1;
		distance = 0;

		while (status == 1) {
			Delay(10);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) {
				distance++;
			} else {
				status = 0;
			}
//			if(distance>70)
//			{
//				status=0;
//				distance=defVal;
//			}
		}
	}
	return (distance);
}
uint16_t ultraSonicL() {
	uint8_t status = 0;
	uint32_t distance = defVal;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_Delay(0.001);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(0.01);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) {
		status = 1;
		distance = 0;

		while (status == 1) {
			Delay(10);
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)) {
				distance++;
			} else {
				status = 0;
			}
		}
	}
	return (distance);
}
uint16_t ultraSonicB() {
	uint8_t status = 0;
	uint32_t distance = defVal;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(0.001);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(0.01);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
		status = 1;

		while (status == 1) {
			Delay(10);
			//HAL_Delay(1);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
				distance++;
			} else {
				status = 0;
			}
		}
	}
	return (distance);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
