
/********************************************************************************
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "header.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Status_Door_Lock = 0; // 0 - Door lock is CLOSE; 1 - Door lock is open
uint8_t counter = 0;
uint8_t ErrorCounter = 0;
uint8_t Flag_Rx_Full = 0;
uint8_t Flag_Start_Collect = 0;
uint8_t Tcounter = 0;
uint8_t Tcounter1 = 0; // For clear WatchDog and flashing Led one time per second
//uint8_t Tcounter2 = 0;
uint16_t size_UART;
uint8_t Data_Rx[32];
uint8_t Data_Tx[32];
uint8_t keys[][10]={
	{0x30,0x42,0x30,0x30,0x33,0x45,0x44,0x35,0x44,0x32},//0 true
	{0x36,0x42,0x30,0x30,0x38,0x30,0x46,0x32,0x36,0x37},//1 true
	{0x35,0x36,0x35,0x41,0x45,0x38,0x30,0x43,0x36,0x36},//2	
	{0x35,0x36,0x35,0x41,0x45,0x38,0x30,0x43,0x37,0x35},//3	
	{0x35,0x36,0x35,0x41,0x45,0x38,0x30,0x48,0x36,0x35},//4	
	{0x35,0x36,0x35,0x41,0x45,0x38,0x30,0x43,0x36,0x35} //5	true
};
	uint8_t access=0;
	uint8_t a = 0;	
	uint8_t str[14];//=0;
	
	
uint8_t transmitBuffer[32];
uint8_t receiveBuffer[32];	
	
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void Short_Buzzer_Beep(uint8_t x);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void MX_NVIC_Init(void);
void Door_Lock_OPENING(void);
void Door_Lock_CLOSING(void);
void Get_Status_Door_Lock(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void TIM3_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim3);
	Tcounter ++;	
	Tcounter1 ++;
		if (Tcounter1 >= 10) {
			HAL_IWDG_Refresh(&hiwdg);
			GPIOC->ODR^=(LED_on_board_Pin);			
			Tcounter1 = 0;	
		}		
}
void TIM2_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim2);
	CLK_GPIO_Port->ODR^=(CLK_Pin);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
	MX_NVIC_Init();	
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();	
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);
	MX_IWDG_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LED_RFID_OFF;	
	BUZZER_FLASH_OFF;

		size_UART = sprintf((char *)Data_Tx,"Hello! I`m alive\n\r");
		HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);

	if (Door_Lock_Close == 0 && Door_Lock_Open == 1){
		Status_Door_Lock = 0; // Door lock is broken
		Door_Lock_OPENING();
	}
	if (Door_Lock_Close == 1 && Door_Lock_Open == 0){
		Status_Door_Lock = 0; // Door lock is CLOSE
		Door_Lock_OPENING();
	}

	Get_Status_Door_Lock();	
	
	HAL_UART_Receive_IT(&huart1,(uint8_t*)str, 14);
////////////////////////////////////////////////////////////////////////////////////////////////////////	
  while (1){
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
																																					//***********************
		if(huart1.RxXferCount==0){																										//
			if (str[0] == 0x02){								// If I found start symbol 0x02							//
				Flag_Start_Collect = 1;				// I set flag start collect									//
			}	else {																																		//			
				huart1.RxXferCount = 13;
				for (uint8_t w = 0; w <=13; w++){
					str[w] = 0;
				}			
			}	
				if (Flag_Start_Collect == 1){																								//
				for (uint8_t w = 1; w <=12; w++){																					// Ignore start symbol 0x02	
					Data_Rx[w-1] = str[w];																									//
					if (w == 12){
						Flag_Rx_Full = 1;						// If buffer is full - set flag buffer full	//
						__HAL_UART_DISABLE(&huart1);
						Flag_Start_Collect = 0;																									//											
					}
				}																																					// Recive byte and feeling buffer Data_Rx
			}																																						//
			for (uint8_t w = 0; w <=13; w++){
				str[w] = 0;
			}
			
			
			__HAL_UART_CLEAR_OREFLAG(&huart1);
			
//			HAL_UART_Transmit(&huart2,str, 14, 0xFFFF);	
//			size_UART = sprintf((char *)Data_Tx,"\n\r");
//			HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);	
		
			HAL_UART_Receive_IT(&huart1,(uint8_t*)str, 14);// Start UART Recive again						//HAL_NVIC_SystemReset();	
		}																																							//
//***********************//***********************//***********************//***********************
		if (Flag_Rx_Full == 1){																												//
			Short_Buzzer_Beep(1);																												//
			__HAL_UART_DISABLE(&huart1);																								//
			Flag_Rx_Full = 0;																														//
			access = 1;														//	Access denided										//
//			HAL_UART_Transmit(&huart2, Data_Rx, 11, 0xFFFF);														//
			a = sizeof(keys)/10;																												//
			for (uint8_t i = 0; i<=a-1; i++){																						//
				for (uint8_t q = 0; q<=9; q++){																						//
					if (keys[i][q]==Data_Rx[q]){																						// Compare keys and set flag access
						counter ++;																														//
						if (counter == 10){access=2;a=i;}	// Access granted										//		
					}																																				//
				}																																					//
				counter = 0;																															//
			}																																						//
			for (uint8_t w = 0; w<=13; w++){																						// Clear all flag and buffer
				Data_Rx[w]=0;																															//
			}																																						//
			HAL_Delay(200);																														//
			__HAL_UART_ENABLE(&huart1);																									//
		}																																							//
//***********************//***********************//***********************//***********************		
		if (access == 2){																															//
			size_UART = sprintf((char *)Data_Tx,"\n\rAccess granted %d \n\r", a);				//
			HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);											//
			access = 0;			//	Clear Access flag																				//
			a = 0;																																			//
			Door_Lock_OPENING();																												//
		}	else if (access == 1){																											//
			access=0;			//	Clear Access flag																					//
			a = 0;																																			//
			Short_Buzzer_Beep(2);
			ErrorCounter ++;
			if (ErrorCounter >= 5){
				ErrorCounter = 0;
				size_UART = sprintf((char *)Data_Tx,"\n\rToo mamy time.. Pause 10 sek\n\r");//
				HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);
				HAL_Delay(10000);
			}
			size_UART = sprintf((char *)Data_Tx," Access denided\n\r");									//
			HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);											//
			HAL_Delay(200);																															//
		}																																							//
//***********************Door_Lock_Close_Button****************************************************			
		if (Door_Lock_Close_Button ==1){
			if(Status_Door_Lock == 0){
				size_UART = sprintf((char *)Data_Tx,"Bad command. Doorlock have been closed\n\r");
				HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);					
				break;
			}

			size_UART = sprintf((char *)Data_Tx,"Door will be closing...\n\r");
			HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);	
			Short_Buzzer_Beep(15);						

			Door_Lock_CLOSING();
			Short_Buzzer_Beep(3);
		}	
//********************************************************************************************		
  }
  /* USER CODE END 3 */
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;//|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void){

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void){

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void){							// for RFID

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void){							// for PC

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void){

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIOC pin Output Level */
  HAL_GPIO_WritePin(LED_on_board_GPIO_Port, LED_on_board_Pin, GPIO_PIN_RESET);

  /*Configure GPIOA pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RFID_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, ENABLE_Pin | CLK_Pin | CW_CCW_Pin | LED_RFID_Pin | BUZZER_FLASH_Pin | Door_Lock_Close_Button_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_on_board_Pin */
  GPIO_InitStruct.Pin = LED_on_board_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_on_board_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE_Pin CLK_Pin CW_CCW_Pin LED_RFID_Pin LED_FLASH_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin|CLK_Pin|CW_CCW_Pin|LED_RFID_Pin|BUZZER_FLASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 | PA11 | PA12*/
  GPIO_InitStruct.Pin = Door_Lock_Close_Pin|Door_Lock_Open_Pin|Door_Lock_Close_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MX_NVIC_Init(void){
  /* TIM2_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(TIM2_IRQn, 3, 3);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
		HAL_NVIC_SetPriority(TIM3_IRQn, 3, 3);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USART interrupt configuration */	
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);	
		HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ(USART2_IRQn);	
	
}

void Door_Lock_OPENING(void){
	if (Status_Door_Lock == 0){							//Status 0 - Door lock is CLOSE; 1 - Door lock is open
		Tcounter = 0;
		while (Door_Lock_Open == 0){
			BUZZER_FLASH_ON; 										// PinA 7
			LED_RFID_ON; 												// Set LED_RFID			
			ENABLE_ON;
			CW_CCW_UP;
			if (Tcounter >= 50){
				ENABLE_OFF;
				BUZZER_FLASH_OFF; 								// Pin A7
				size_UART = sprintf((char *)Data_Tx,"Door lock is broken, not respone\n\r");
				HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);
				break;
			}
		}
		if (Door_Lock_Close == 1 && Door_Lock_Open == 1){
			Status_Door_Lock = 1;
			ENABLE_OFF;
			size_UART = sprintf((char *)Data_Tx,"Door opened\n\r");
			HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);
			HAL_Delay (1000);
			BUZZER_FLASH_OFF;
			LED_RFID_OFF;												// Reset LED_RFID						
		}
	}
	Get_Status_Door_Lock();
}

void Door_Lock_CLOSING(void){
	if (Status_Door_Lock == 1){
		Tcounter = 0;
		while (Door_Lock_Close == 1){
			BUZZER_FLASH_ON; 										// PinA 7
			LED_RFID_ON; 												// Set LED_RFID				
			ENABLE_ON;
			CW_CCW_DOWN;
			if (Tcounter >= 50){
				ENABLE_OFF;
				BUZZER_FLASH_OFF; 								// Pin A7
				size_UART = sprintf((char *)Data_Tx,"Door lock is broken, not respone\n\r");
				HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);				
				break;
			}
		}
		if (Door_Lock_Close == 0 && Door_Lock_Open == 0){
			Status_Door_Lock = 0;
			ENABLE_OFF;
			size_UART = sprintf((char *)Data_Tx,"Door closed\n\r");
			HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);
			HAL_Delay (1000);
			BUZZER_FLASH_OFF;
			LED_RFID_OFF;												// Reset LED_RFID					
		}
	}	
	Get_Status_Door_Lock();
}

void Get_Status_Door_Lock(void){
	if (Door_Lock_Close == 0 && Door_Lock_Open == 0){
		Status_Door_Lock = 0; // Door lock is CLOSE
	}
	if (Door_Lock_Close == 1 && Door_Lock_Open == 1){
		Status_Door_Lock = 1; // Door lock is OPEN
	}	
		if (Status_Door_Lock == 1){
		size_UART = sprintf((char *)Data_Tx,"Door lock is OPEN\n\r");
		HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);
	}else{
		size_UART = sprintf((char *)Data_Tx,"Door lock is CLOSE\n\r");
		HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);		
	}
}

void Short_Buzzer_Beep(uint8_t x){
 for (uint8_t q = 1; q <= x; q++){
		BUZZER_FLASH_ON;
		HAL_Delay(300);
		BUZZER_FLASH_OFF;	
		HAL_Delay(300);		
	}
}

/* IWDG init function */
static void MX_IWDG_Init(void){
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
