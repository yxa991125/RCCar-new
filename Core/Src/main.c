/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

//bsp
#include "bsp_oled.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_adc.h"
#include "bsp_flash.h"

#include "hall_speed.h"
#include "servo_basic_control.h"
#include "robot_select_init.h"

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

static uint8_t HardWareVersion = HW_UnKnown; //绯荤粺纭欢鐗堟湰

uint8_t BlueToothBuffer = 0;//钃濈墮涓插彛鎺ユ敹缂撳啿
uint8_t rosbuffer = 0;      //ROS涓插彛鎺ユ敹缂撳啿鍖?
uint8_t usart1_buffer = 0;  //涓插彛1鎺ユ敹缂撳啿鍖?
uint8_t rs485_buffer = 0;   //485鎺ユ敹缂撳啿鍖?

DebugType_t g_sys_debug = { 0 }; //鍏ㄥ眬璋冭瘯鍙橀噺
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  
  /* Initialize Ackermann servo/ESC control state */
  ServoBasic_Init();
	
		//DWT瀹氭椂鍣ㄥ垵濮嬪寲,鐢ㄤ簬瀹炵幇ms銆乽s寤舵椂鍑芥暟
		DWT_Init();

		//鍒濆鍖栭湇灏旇疆閫熺姸鎬侊紝璇ュ姛鑳藉湪 PE9/PE11 EXTI 杈撳叆涓婂伐浣?
		HallSpeed_Init();

		//ADC鐢ㄦ埛閰嶇疆鍒濆鍖?
		ADC_Userconfig_Init();
	
	//鍚姩FreeRTOS绯荤粺璋冭瘯瀹氭椂鍣?
	HAL_TIM_Base_Start(&htim6);
	
	//OLED灞忓箷鍒濆鍖?
	pOLEDInterface_t oled = &UserOLED;
	oled->init();
	
	//C63X纭欢鐗堟湰纭畾(閫氳繃鐗堟湰io鍙?纭纭欢鐗堟湰)
	uint8_t Version = 0;
	Version |= (HAL_GPIO_ReadPin(VersionBit2_GPIO_Port,VersionBit2_Pin))<<2;
	Version |= (HAL_GPIO_ReadPin(VersionBit1_GPIO_Port,VersionBit1_Pin))<<1;
	Version |= (HAL_GPIO_ReadPin(VersionBit0_GPIO_Port,VersionBit0_Pin))<<0;

	if( Version == 0 )
	{
		HardWareVersion = HW_1_1;
	}
	else if( Version == 7)
	{
		HardWareVersion = HW_1_0;
		
		//鍙嶅垵濮嬪寲V2.0鐗堟湰鐨凜AN寮曡剼
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);
		
		//鍒濆鍖朧1.0鐗堟湰CAN寮曡剼
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		__HAL_RCC_GPIOA_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		//V1.0鐗堟湰鎬ュ仠寮€鍏?
		HAL_GPIO_DeInit(ENKey_GPIO_Port,ENKey_Pin);
		__HAL_RCC_GPIOD_CLK_ENABLE();
		GPIO_InitStruct.Pin = ENKey_V1_0_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(ENKey_V1_0_GPIO_Port, &GPIO_InitStruct);
		
		//V1.0鐗堟湰鎸夐敭
		HAL_GPIO_DeInit(UserKey_V1_0_Port,UserKey_V1_0_Pin);
		HAL_GPIO_DeInit(UserKey_GPIO_Port,UserKey_Pin);
		__HAL_RCC_GPIOD_CLK_ENABLE();
		GPIO_InitStruct.Pin = UserKey_V1_0_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(UserKey_V1_0_Port, &GPIO_InitStruct);
	}
	else
	{
		oled->ShowString(0,0,"unknown hardware-version.");
		oled->ShowString(0,35,"please reset and-retry.");
		oled->RefreshGram();
		while(1)
		{
			HAL_GPIO_TogglePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin);
			HAL_Delay(500);
		}
	}
	
	//寮€鍚埅妯￠仴鎺х浉鍏冲畾鏃跺櫒鎹曡幏鍔熻兘
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
	
	//鍚姩涓插彛鎺ユ敹涓柇
	HAL_UART_Receive_IT(&huart2,&BlueToothBuffer,1);
	HAL_UART_Receive_IT(&huart4,&rosbuffer,1);
	HAL_UART_Receive_IT(&huart1,&usart1_buffer,1);
	HAL_UART_Receive_IT(&huart3,&rs485_buffer,1);
	
	//485鎺ュ彛娴嬭瘯浣跨敤
//	uint8_t _485debug[] = {"hello c63x! RS485 Test Info"};
//	HAL_UART_Transmit(&huart3,_485debug,sizeof(_485debug)/sizeof(_485debug[0]),500);
	
	//閫氳繃鐢典綅鍣ㄧ‘璁よ溅鍨?
	HAL_Delay(500);
	Robot_Select();
	
	//璇诲彇鐢ㄦ埛淇濆瓨鐨勯粯璁ら€熷害鍊?
	uint32_t flashdata[2];
	User_Flash_ReadParam((uint32_t *)flashdata,2);
	
	//灏忚溅榛樿閬ユ帶閫熷害鍊?
	if( flashdata[0]!=0xFFFFFFFF )
	{
		if( flashdata[0]<=3500 )
			RobotControlParam.defalutSpeed = flashdata[0];
	}
	
	//绾犲亸绯绘暟鍊?
	if( flashdata[1]!=0xFFFFFFFF )
	{
		if( flashdata[1]<=100 )
			RobotControlParam.LineDiffParam = flashdata[1];
	}
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

UART_HandleTypeDef *DebugSerial = &huart1;

//printf鍑芥暟瀹炵幇
int fputc(int ch,FILE* stream)
{
	while( HAL_OK != HAL_UART_Transmit(DebugSerial,(const uint8_t *)&ch,1,100));
	return ch;
}

//鑾峰彇纭欢鐗堟湰
uint8_t get_HardWareVersion(void)
{
	return HardWareVersion;
}

//鑾峰彇鎬ュ仠寮€鍏崇姸鎬?涓庣増鏈浉鍏?
GPIO_PinState get_EnKeyState(void)
{
	if( HardWareVersion == HW_1_0 )
	{
		return HAL_GPIO_ReadPin(ENKey_V1_0_GPIO_Port,ENKey_V1_0_Pin);
	}
	else if( HardWareVersion == HW_1_1 )
	{
		return HAL_GPIO_ReadPin(ENKey_GPIO_Port,ENKey_Pin);
	}
	
	return GPIO_PIN_RESET;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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

