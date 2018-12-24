#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include <inttypes.h>
#include "main.h"
//#include "flash.h"

#include "stm32f1xx_hal.h"

#define ENABLE_ON			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin,	GPIO_PIN_SET)			//PIN_0
#define ENABLE_OFF		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin,	GPIO_PIN_RESET)

#define CW_CCW_UP			HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin,	GPIO_PIN_SET)			//PIN_4
#define CW_CCW_DOWN		HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin,	GPIO_PIN_RESET)

#define Door_Lock_Close_Button		HAL_GPIO_ReadPin(Door_Lock_Close_Button_Port, Door_Lock_Close_Button_Pin)		//pin 12

#define Door_Lock_Open		HAL_GPIO_ReadPin(Door_Lock_Open_Port, Door_Lock_Open_Pin)		//pin 11 // �������� ��������� �����. ���� �� ���� 1 �� ����� ��������� ������
#define Door_Lock_Close		HAL_GPIO_ReadPin(Door_Lock_Close_Port, Door_Lock_Close_Pin)	//pin 8  // �������� ��������� �����. ���� �� ���� 0 �� ����� ��������� ������
																																															 // 00 - ����� ��������� ������
																																															 // 11 - ����� ��������� ������
																																															 // 10 - � ���������� ���������, ����� �� ��������� ������ ��� ������
																																															 // 01 - ����������� ���������, �������� ����� ������

#define BUZZER_FLASH_ON		HAL_GPIO_WritePin(BUZZER_FLASH_GPIO_Port, BUZZER_FLASH_Pin,	GPIO_PIN_SET)    //PIN_7
#define BUZZER_FLASH_OFF	HAL_GPIO_WritePin(BUZZER_FLASH_GPIO_Port, BUZZER_FLASH_Pin,	GPIO_PIN_RESET)

#define LED_RFID_OFF	HAL_GPIO_WritePin(LED_RFID_GPIO_Port, LED_RFID_Pin,	GPIO_PIN_SET)			//PIN_6
#define LED_RFID_ON		HAL_GPIO_WritePin(LED_RFID_GPIO_Port, LED_RFID_Pin,	GPIO_PIN_RESET)

