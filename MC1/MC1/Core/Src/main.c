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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "LCD.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define BUFFER_SIZE  3

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

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern int col;
extern int row;
extern uint8_t wave;
extern double adc_value;
extern int voltage;
extern int wave_length;
extern int frequency;
extern uint8_t tx_buffer[BUFFER_SIZE];
extern uint8_t wave_length_raw_voltage;
extern uint8_t frequency_raw_voltage;
extern int prev_time;
extern int period;

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);

void init_clocks(void);
void SysTick_Handler(void);
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void Init_GPIO(void);
void get_row(void);
void read_keypad(void);
void get_wave(void);
void wait_for_input(void);
static void MX_SPI1_Init(void);


int col = 0;
int row =0;
uint8_t wave=0;
double adc_value;
int voltage;
uint8_t wave_length_raw_voltage;
uint8_t frequency_raw_voltage;
int wave_length;
int frequency;
uint8_t tx_buffer[BUFFER_SIZE];
uint8_t rx_buffer[1]={0};
char wave_type [50];
int prev_time =0;
int period = 0;


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
	init_clocks();
	SystemCoreClockUpdate();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();

	LCD_Init();
	
	LCD_Puts(0, 0, "99243028");
	LCD_Puts(0, 1, "99243061");
	HAL_Delay(1000);
	LCD_Clear();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


		
    /* USER CODE BEGIN 3 */
		prev_time = 0;
		period = 0;
		LCD_Clear();
		LCD_Puts(0, 0, "1:sine 2:square 3:sawooth");
		LCD_Puts(0, 1, "4:traiangle 5:stairs 6:abs sine");
		get_wave();
		char data[50];
		LCD_Clear();
		LCD_Puts(0,0,"select wave time");
		wait_for_input();
		LCD_Clear();
		HAL_ADC_Start(&hadc1); 
		while(HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK);
		adc_value = HAL_ADC_GetValue(&hadc1);
		voltage = round((adc_value*100)/4095);
		wave_length_raw_voltage = voltage;
		wave_length = (voltage*95)+500;
		sprintf(data, "%u was selected as wave time", wave_length);  
		LCD_Puts(0,0,data);
		HAL_Delay(500);
		LCD_Clear();
		LCD_Puts(0,0,"select frequency");
		wait_for_input();
		LCD_Clear();
		HAL_ADC_Start(&hadc1); 
		while(HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK);
		adc_value = HAL_ADC_GetValue(&hadc1);
		voltage = round((adc_value*100)/4095);
		frequency_raw_voltage = voltage;
		frequency = voltage*10;
		sprintf(data, "%u was selected as frequency", frequency);  
		LCD_Puts(0,0,data);
		HAL_Delay(500);
		LCD_Clear();
		
		tx_buffer[0] = wave;
		tx_buffer[1] = wave_length_raw_voltage;
		tx_buffer[2] = frequency_raw_voltage;
		
		HAL_SPI_Transmit(&hspi1, tx_buffer, 3, HAL_MAX_DELAY);
		MX_TIM2_Init();
		char str[50];
		sprintf(str, "  time: %d freq: %d", wave_length, frequency);
		strcat(wave_type,str);
		LCD_Puts(0,0,wave_type);
		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {}
		HAL_SPI_Receive(&hspi1,rx_buffer, 1, 100);
		HAL_Delay(1);
		int estimated_frequency = 2*((100000)/period);
		if ( abs(estimated_frequency - (2* frequency)) <frequency/20)
		{
			estimated_frequency =  estimated_frequency/2;
		}
		
		char estimated_frequency_str[50];
		sprintf(estimated_frequency_str, "  estimated_frequency: %d ", estimated_frequency);
		LCD_Clear();
		LCD_Puts(0,0,estimated_frequency_str);
		HAL_TIM_Base_Stop(&htim2);
		wait_for_input();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void init_clocks(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_ChannelConfTypeDef sConfig;

	GPIO_InitStruct.Pin = GPIO_PIN_0; 
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;  
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	__HAL_RCC_ADC1_CLK_ENABLE(); 

	hadc1.Instance = ADC1;

	sConfig.Channel = ADC_CHANNEL_0; 
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);


}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

	__HAL_RCC_TIM2_CLK_ENABLE();
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 80 - 1; // Set the prescaler value
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP; // Set the counter mode
	htim2.Init.Period = 100000; // Set the period value
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Set the clock division
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Enable auto-reload preload
	TIM2->DIER |= TIM_DIER_UIE;
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_0 ;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	// Enable the interrupt for PB7
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void EXTI9_5_IRQHandler(void) {
    // Check if interrupt was triggered by PB7
    if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_7) != RESET) {
        // Clear interrupt flag
        __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_7);

				period = TIM2 ->CNT - prev_time;
				if (TIM2 ->CNT ==0)
				{
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
				}
				prev_time = TIM2 ->CNT;
    }
}

/* USER CODE BEGIN 4 */
void read_keypad(void)
{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
	while (1)
	{
		col=0;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
		HAL_Delay(25);
		get_row();
		if (row!=0)
		{
			col=1;
			return;
		}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
		HAL_Delay(25);
		get_row();
		if (row!=0)
		{
			col=2;
			return;
		}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
		HAL_Delay(25);
		get_row();
		if (row!=0)
		{
			col=3;
			return;
		}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
	}
}

void get_row(void)
{
		GPIO_PinState input_state;
		row=0;
    input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
		if (input_state ==GPIO_PIN_SET)
		{
			row=1;
			return;
		}
    input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
		if (input_state ==GPIO_PIN_SET)
		{
			row=2;
			return;
		}
    input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
		if (input_state ==GPIO_PIN_SET)
		{
			row=3;
			return;
		}
    input_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
		if (input_state ==GPIO_PIN_SET)
		{
			row=4;
			return;
		}
}

void get_wave(void)
{
		read_keypad();
		LCD_Clear();
		switch(col)
		{
			case 1:
				if (row==1)
				{
					wave = 1;
					strcpy(wave_type, "sine");
					LCD_Puts(0,0,"sine wave selected");
				}
				else
				{
					wave = 4;
					strcpy(wave_type, "triangle");
					LCD_Puts(0,0,"triangle wave selected");
				}
				break;
			case 2:
				if (row==1)
				{
					wave = 2;
					strcpy(wave_type, "square");
					LCD_Puts(0,0,"square wave selected");
				}
				else
				{
					wave = 5;
					strcpy(wave_type, "stair");
					LCD_Puts(0,0,"stair wave selected");
				}
					break;
			case 3:
				if (row == 1)
				{
					wave = 3;
					strcpy(wave_type, "ramp");
					LCD_Puts(0,0,"sawtooth wave selecred");
				}
				else
				{
					wave = 6;
					strcpy(wave_type, "abs sin");
					LCD_Puts(0,0,"abs sin wave selecred");
				}

				break;
		}
		HAL_Delay(1000);
}
void wait_for_input(void)
{
	while(1)
	{
	read_keypad();
	if (row==4 && col==3)
		return;
	}
}

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
