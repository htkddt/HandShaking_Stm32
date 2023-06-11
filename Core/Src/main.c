/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int BUFF_SIZE = 18;

uint8_t nRxData[18];
uint8_t nTxData[18];
uint8_t strCommand[4];
uint8_t strOpt[3];
uint8_t strData[8];
uint8_t Rx_Index = 0;
uint8_t Tx_Index = 0;
uint8_t Rx_flag = 0;
uint8_t Rx_data;
uint8_t data[8];

uint8_t STX[] = {0x02};													
uint8_t ETX[] = {0x03};													
uint8_t ACK[] = {0x06};													
uint8_t SYN[] = {0x16};	

uint8_t Closed[] = {0x43, 0x4C, 0x4F, 0x53};

uint8_t MOVL[] = {0x4D, 0x4F, 0x56, 0x4C};	
uint8_t GPOS[] = {0x47, 0x50, 0x4F, 0x53};			
uint8_t GSTT[] = {0x47, 0x53, 0x54, 0x54};					
uint8_t GVEL[] = {0x47, 0x56, 0x45, 0x4C};			
uint8_t NUL[] = {0x4E, 0x55, 0x4C, 0x4C};	

bool bDataAvailable = false;

uint8_t eLED[] = {0x65, 0x4C, 0x45, 0x44};			
uint8_t dLED[] = {0x64, 0x4C, 0x45, 0x44};
uint8_t bONN[] = {0x62, 0x4F, 0x4E, 0x4E};
uint8_t	bOFF[] = {0x62, 0x4F, 0x46, 0x46};
uint8_t bTOG[] = {0x62, 0x54, 0x4F, 0x47}; 
bool bLed = false;
bool bToggle = false;

uint8_t eBTN[] = {0x65, 0x42, 0x54, 0x4E};
uint8_t dBTN[] = {0x64, 0x42, 0x54, 0x4E};
uint8_t Btn_SingleClick[] = {0x53, 0x69, 0x6E, 0x67, 0x63, 0x6C, 0x69, 0x63};
uint8_t Btn_LongClick[] = {0x4C, 0x6F, 0x6E, 0x67, 0x63, 0x6C, 0x69, 0x63};
uint8_t Present_State_Btn;
uint8_t Next_State_Btn;
bool bBtn = false;
bool bSingleClick = false;
bool bLongClick = false;

uint8_t eLIG[] = {0x65, 0x4C, 0x49, 0x47};
uint8_t dLIG[] = {0x64, 0x4C, 0x49, 0x47};
uint8_t rLIG[] = {0x52, 0x55, 0x4E};
uint8_t sLIG[] = {0x53, 0x54, 0x4F};
uint16_t ADC_value;
char Light_Sensor[8];
bool bLight = false;

uint8_t ePWM[] = {0x65, 0x50, 0x57, 0x4D};
uint8_t dPWM[] = {0x64, 0x50, 0x57, 0x4D};	
uint8_t bSET[] = {0x53, 0x45, 0x54};
uint8_t Duty;
int32_t counter_speed, counter_speed_last;
int CCR, ARR;
float speed;
bool bPwm = false;

bool StrCompare(uint8_t *pBuff, uint8_t *Sample, uint8_t nSize)
{
	for (int i=0; i< nSize; i++)
	{
		if (pBuff[i] != Sample[i])
		{
			return false;
		}
	}
	return true;
}

uint8_t *subString(uint8_t *s, int pos, int index)
{
	memset(data,0,sizeof(data));
	int j = pos;
	int k = pos + index;
	for (int i = pos; i < k; i++)
	{
		data[i-j] = s[i];
	}
	return data;
}

bool ReadComm(uint8_t *pBuff, uint8_t nSize)
{
	if ((pBuff[0] == STX[0]) && (pBuff[17] == ETX[0]))
	{
		memcpy(strCommand, subString(pBuff, 1, 4), 4);
		memcpy(strOpt, subString(pBuff, 5, 3), 3);
		memcpy(strData, subString(pBuff, 8, 8), 8);
		
		bDataAvailable = true;
	}
	else
	{
		bDataAvailable = false;
	}
	return bDataAvailable;
}

bool WriteComm(uint8_t *pBuff, uint8_t nSize)
{
	return HAL_UART_Transmit(&huart1, pBuff, nSize, 1000);
}
void SerialProcess(void)
{
	Tx_Index = 0;
	if(StrCompare(strCommand, MOVL, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);

		WriteComm(nTxData, BUFF_SIZE);
	}
	else if (StrCompare(strCommand, GPOS, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);

		WriteComm(nTxData, BUFF_SIZE);
	}	
	else if (StrCompare(strCommand, GVEL, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);

		WriteComm(nTxData, BUFF_SIZE);
	}		
	else if (StrCompare(strCommand, GSTT, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);

		WriteComm(nTxData, BUFF_SIZE);
	}	
	else if(StrCompare(strCommand, eLED, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
	}
	else if(StrCompare(strCommand, dLED, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		bLed = false;
		bToggle = false;
	}
	else if(StrCompare(strCommand, bONN, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		bLed = true;
		bToggle = false;
	}
		else if(StrCompare(strCommand, bOFF, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		bLed = false;
		bToggle = false;
	}
	else if(StrCompare(strCommand, bTOG, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		bToggle = true;
		bLed = false;
	}
	else if(StrCompare(strCommand, eBTN, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		bBtn = true;
	}
	else if(StrCompare(strCommand, dBTN, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		bBtn = false;
	}
	else if(StrCompare(strCommand, eLIG, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		HAL_Delay(100);
		
		if(StrCompare(strOpt, rLIG, 3))
			bLight = true;
		else if(StrCompare(strOpt, sLIG, 3))
			bLight = false;
	}
	else if(StrCompare(strCommand, dLIG, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		bLight = false;
	}
	else if(StrCompare(strCommand, ePWM, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
		
		if(StrCompare(strOpt, bSET, 3))
		{
			bPwm = true;
			Duty = atoi((char *)strData);
		}
	}
	else if(StrCompare(strCommand, dPWM, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		bPwm = false;
		
		WriteComm(nTxData, BUFF_SIZE);
	}
	else if(StrCompare(strCommand, Closed, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		bLed = false;
		bToggle = false;
		bBtn = false;
		bLight = false;
		bPwm = false;
	}
	else
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, NUL, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);
		
		WriteComm(nTxData, BUFF_SIZE);
	}
}
void ButtonProcess(void)
{
	Present_State_Btn = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	HAL_Delay(10);
	Next_State_Btn = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	if(Present_State_Btn == 0 && Next_State_Btn == 0)
	{
		HAL_Delay(500);
		Next_State_Btn = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
		if(Next_State_Btn == 0)
		{
			bLongClick = true;
		}
		else
		{
			bLongClick = false;
		}
		if(bLongClick == false)
		{
			memcpy(strData,Btn_SingleClick,8);
			SerialProcess();
		}
		else
		{
			while(Next_State_Btn == 0)
			{
				Next_State_Btn = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
			}
			memcpy(strData,Btn_LongClick,8);
			SerialProcess();
		}
	}
}
void LightSensorProcess(void)
{
	memset(Light_Sensor,0,sizeof(Light_Sensor));
	memset(strData,0,sizeof(strData));
	ADC_value = HAL_ADC_GetValue(&hadc1);
	sprintf(Light_Sensor,"%d\n",ADC_value);
	memcpy(strData,Light_Sensor,8);
	SerialProcess();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	if(htim->Instance == TIM3)
//	{		
//		if(bPwm == true)
//		{
//			counter_speed = TIM2->CNT - counter_speed_last;
//			counter_speed_last = TIM2->CNT;
//		}
//		else if(bPwm == false)
//		{
//			TIM2->CNT = 0;
//			counter_speed_last = 0;
//		}
//	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		if(Rx_data != 0x03)
			nRxData[Rx_Index++] = Rx_data;
		else if(Rx_data == 0x03)
		{
			nRxData[17] = Rx_data;
			Rx_flag = 1;
		}
		HAL_UART_Receive_IT(&huart1,&Rx_data,1);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1,&Rx_data,1);
	HAL_ADC_Start(&hadc1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Rx_flag)
		{
			ReadComm(nRxData, BUFF_SIZE);
			Rx_flag = 0;
			Rx_Index = 0;
		}
		if(bDataAvailable == true)
		{
			SerialProcess();
			bDataAvailable = false;
		}
		if(bLed == true)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
		else if(bToggle == true)
		{
			while(!Rx_flag)
			{
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
				HAL_Delay(500);
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		if(bBtn == true)
		{
			ButtonProcess();
		}
		if(bLight == true)
		{
			LightSensorProcess();
		}
		if(bPwm == true)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			ARR = TIM1->ARR;
			CCR = Duty*ARR/100;
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCR);
			counter_speed = TIM2->CNT - counter_speed_last;
			counter_speed_last = TIM2->CNT;
			speed = (float)counter_speed*(60000)/(50*1950);
		}
		else
		{
			Duty = 0;
			ARR = TIM1->ARR;
			CCR = Duty*ARR/100;
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCR);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			TIM2->CNT = 0;
			counter_speed_last = 0;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
