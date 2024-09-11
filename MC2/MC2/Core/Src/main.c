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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#define BUFFER_SIZE  3

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern int col;
extern int row;
extern uint8_t wave;
extern uint8_t raw_voltage_wave_length;
extern uint8_t raw_voltage_frequency;
extern int wave_length;
extern int frequency;
extern uint8_t rx_buffer[BUFFER_SIZE];
extern bool finished;
extern bool first_time;
extern uint16_t ramp_value;
extern uint16_t triangle_wave_value;
extern bool triangle_wave_value_increasing;
extern uint16_t prev_triangle_wave_value;
extern uint16_t sine_wave_value;
extern uint16_t prev_sine_wave_value;
extern bool sine_wave_value_increasing;
extern bool stair_wave_value_increasing;
extern bool ramp_flag;
extern uint16_t stair_wave_value;
extern uint16_t prev_stair_wave_value;
extern int prev_ramp_value;
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
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;


uint16_t ramp_value;
uint16_t triangle_wave_value;
uint16_t prev_triangle_wave_value;
uint8_t wave;
uint8_t raw_voltage_wave_length;
uint8_t raw_voltage_frequency;
uint16_t sine_wave_value;
uint16_t prev_sine_wave_value;
uint16_t stair_wave_value;
uint16_t prev_stair_wave_value;
int prev_ramp_value;
int wave_length;
int frequency;
uint8_t rx_buffer[BUFFER_SIZE]={0,0,0};
bool finished = false;
bool first_time = false;
bool triangle_wave_value_increasing = true;
bool sine_wave_value_increasing=true;
bool stair_wave_value_increasing=false;
bool ramp_flag=false;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void init_clocks(void);
void SystemClock_Config(void);
void Init_GPIO(void);
static void SPI1_Init(void);
void DMA_Init(void);
void sine_wave(void);
void abs_sine_wave(void);
void square_wave(void);
void timer_Init(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void ramp_wave(void);
void Time_Init(void);
void triangle_wave(void);
void stair_wave(void);
void timer4_Init(void);
uint32_t get_shifted(uint32_t,uint32_t );
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
	SystemClock_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  //MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */
				ramp_value =0;
			 triangle_wave_value=0;
			 prev_triangle_wave_value=0;
			 wave=0;
			 raw_voltage_wave_length=0;
			 raw_voltage_frequency=0;
			 sine_wave_value=0;
			 prev_sine_wave_value=0;
			 stair_wave_value=0;
			 prev_stair_wave_value=0;
			 wave_length=0;
			 frequency=0;
			prev_ramp_value = 0;
			//uint8_t rx_buffer[BUFFER_SIZE]={0,0,0};
			 finished = false;
			 first_time = false;
			 triangle_wave_value_increasing = false;
			 sine_wave_value_increasing=true;
			 stair_wave_value_increasing=false;
			ramp_flag=false;
			
			HAL_SPI_Receive(&hspi1,rx_buffer, 3, HAL_MAX_DELAY);
			wave = rx_buffer[0];
			raw_voltage_wave_length = rx_buffer[1];
			raw_voltage_frequency = rx_buffer[2];
			wave_length = (raw_voltage_wave_length*95)+500 ;
			frequency = raw_voltage_frequency*10 ;
	
			switch(wave)
			{
				case 1:
					sine_wave();
					break;
				case 2:
					square_wave();
					break;
				case 3:
					ramp_wave();
					break;
				case 4:
					triangle_wave();
					break;
				case 5:
					stair_wave();
					break;
				case 6:
					abs_sine_wave();
					break;
			}
			
			
			uint8_t tx_buffer[1]={1};
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			HAL_Delay(1);
			HAL_SPI_Transmit(&hspi1, tx_buffer, 1, HAL_MAX_DELAY);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			
    /* USER CODE BEGIN 3 */
			
  }
	while(1)
	{
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

void init_clocks(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
}



/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

	__HAL_RCC_SPI1_CLK_ENABLE();
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

	
	__HAL_RCC_TIM5_CLK_ENABLE();    // Enable the clock for Timer 5

htim5.Instance = TIM5;    // Timer 5
htim5.Init.Prescaler = 80 - 1;    
htim5.Init.CounterMode = TIM_COUNTERMODE_UP;    // Up counting mode
htim5.Init.Period = (100000/frequency)-1;    
htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    // No clock division
htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;    // Enable auto-reload preload

HAL_TIM_Base_Init(&htim5);    // Initialize the timer
HAL_TIM_Base_Start(&htim5);    // Start Timer 5
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6| GPIO_PIN_10 | GPIO_PIN_0|GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11| GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
				GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		// Enable the interrupt for PB7
		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

uint16_t fixed_sin(uint16_t current_time)
{
    current_time <<= 1;

    if(current_time == (current_time|0x4000)) 
        current_time = (1<<15) - current_time;
    current_time = (current_time & 0x7FFF) >> 1;
		
		uint32_t current_time_bits = (uint32_t)current_time;

    uint32_t abs_sin_value = (292421UL*(current_time_bits))>>13;
    abs_sin_value = 2746362156UL - ((current_time_bits*abs_sin_value)>>3);
    abs_sin_value = get_shifted(current_time_bits,3370945099UL - ((get_shifted(current_time_bits,get_shifted(current_time_bits,abs_sin_value)))/2));
    return (abs_sin_value+(1UL<<(18)))>>(19); 
		
}

uint32_t get_shifted(uint32_t current_time_bits,uint32_t sin_abs_value)
{
	return current_time_bits * (sin_abs_value>>13);
}

void sine_wave(void)
{
	Time_Init();
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		//sine_wave_value =2048*(sin((TIM2->CNT*frequency*3.14159265358979323846)/100000));
		sine_wave_value= (fixed_sin ((TIM2->CNT*frequency*16384)/100000))/2;
		
		if (sine_wave_value_increasing)
		{
					sine_wave_value = 2048 + sine_wave_value;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		}
		else
		{
					sine_wave_value = 2048 - sine_wave_value;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
	
		if (sine_wave_value>=4090)
		{
			continue;
		}
		if ( abs(sine_wave_value-prev_sine_wave_value)<1200)
			{
				GPIOC -> ODR = sine_wave_value;
				prev_sine_wave_value = sine_wave_value;
			}

		if (finished)
		{
			HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_Base_Stop(&htim3);
			return;
		}
	}
	
}
void abs_sine_wave(void)
{
	Time_Init();
	//MX_TIM5_Init();
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		sine_wave_value= (fixed_sin ((TIM2->CNT*frequency*16384)/100000))/2;
		//sine_wave_value =255*(sin((TIM2->CNT*frequency*3.14)/100000));
		//sine_wave_value =sine_wave_value;
	

	
	if ( abs(sine_wave_value-prev_sine_wave_value)<1200)
		{
			GPIOC -> ODR = sine_wave_value + 2040;
			prev_sine_wave_value = sine_wave_value;
		}

		if (finished)
		{
			HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_Base_Stop(&htim3);
			HAL_TIM_Base_Stop(&htim5);
			return;
		}
	}
}

void square_wave(void)
{
	GPIOC -> ODR = 0x00;
	Time_Init();
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		if (finished)
		{
			HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_Base_Stop(&htim3);
			return;
		}
	}
	
}

void ramp_wave(void)
{
	Time_Init();
	
	
	while(1)
	{
		ramp_value = ((TIM2->CNT*4095*frequency)/100000)/2;
		GPIOC -> ODR = ramp_value + prev_ramp_value;
		if (finished)
		{
			HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_Base_Stop(&htim3);
			
			return;
		}
	}
	
}

void triangle_wave(void)
{
	Time_Init();
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		if (triangle_wave_value_increasing)
		{
			triangle_wave_value = (TIM2->CNT*4095*frequency)/100000;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		}
		
		else
		{
			triangle_wave_value = 4095-((TIM2->CNT*4095*frequency)/100000);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if ( abs(triangle_wave_value-prev_triangle_wave_value)<600)
		{
			GPIOC -> ODR = triangle_wave_value ;
			prev_triangle_wave_value = triangle_wave_value;
		}
		if (finished)
		{
			HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_Base_Stop(&htim3);

			return;
		}
	}
	
}

void stair_wave(void)
	{
	prev_stair_wave_value=0;
	Time_Init();
	timer4_Init();
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		if ( abs(prev_stair_wave_value-stair_wave_value)<500)
		{
			GPIOC -> ODR = stair_wave_value;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if (finished)
		{
			HAL_TIM_Base_Stop(&htim2);
			HAL_TIM_Base_Stop(&htim3);
			HAL_TIM_Base_Stop(&htim4);
			return;
		}
	}
	
	}

void Time_Init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();
	NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 80 - 1; // Set the prescaler value
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP; // Set the counter mode
	htim2.Init.Period = (100000/frequency)-1; // Set the period value
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Set the clock division
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Enable auto-reload preload
	TIM2->DIER |= TIM_DIER_UIE;
	HAL_TIM_Base_Init(&htim2);
	 
	HAL_TIM_Base_Start_IT(&htim2);
	
	__HAL_RCC_TIM3_CLK_ENABLE();
	NVIC_SetPriority(TIM3_IRQn, 0);
  NVIC_EnableIRQ(TIM3_IRQn);
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 4000 - 1; // Set the prescaler value
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP; // Set the counter mode
	htim3.Init.Period = wave_length * 4; // Set the period value
	//htim3.Init.Period = (100000/frequency)-1; // Set the period value
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Set the clock division
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Enable auto-reload preload
	TIM3->DIER |= TIM_DIER_UIE;
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);
}
void timer4_Init(void)
{
	__HAL_RCC_TIM4_CLK_ENABLE();
	NVIC_SetPriority(TIM4_IRQn, 0);
  NVIC_EnableIRQ(TIM4_IRQn);
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 80 - 1; // Set the prescaler value
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP; // Set the counter mode
	htim4.Init.Period = ((100000/frequency)/8)-1; // Set the period value
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Set the clock division
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Enable auto-reload preload
	TIM4->DIER |= TIM_DIER_UIE;
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
}
void TIM2_IRQHandler(void)
{
	
	
    if (TIM2->SR & TIM_SR_UIF) {
			if (wave==1)
			{
				sine_wave_value_increasing = !sine_wave_value_increasing;
			}
			
			else if (wave==2)
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4| GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9 | GPIO_PIN_10| GPIO_PIN_11);
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
			}
			
			else if (wave==3)
			{
					if (ramp_flag)
					{
						ramp_flag = false;
						prev_ramp_value = ramp_value;
					}
					else{
						ramp_flag = true;
						prev_ramp_value = 0;
					}
					ramp_value=0;

			}
			else if (wave==4)
			{
				triangle_wave_value_increasing = !triangle_wave_value_increasing;
			}
			else if (wave==5)
			{
				stair_wave_value_increasing = !stair_wave_value_increasing;
				GPIOC -> ODR = stair_wave_value;
			}
			else if (wave == 6)
			{
			}
			
        TIM2->SR &= ~TIM_SR_UIF;
			
    }
}

void TIM3_IRQHandler(void)
{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		GPIOC -> ODR =  HAL_RCC_GetPCLK1Freq();
			
     if ((TIM3->SR & TIM_SR_UIF) && first_time) {
			 
        TIM3->SR &= ~TIM_SR_UIF;
				finished = true;
			 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
			
    }
		first_time=true;
}

void TIM4_IRQHandler(void)
{
	if (stair_wave_value_increasing)
	{
		prev_stair_wave_value = stair_wave_value;
		stair_wave_value = stair_wave_value + 300;
		if (abs(stair_wave_value-prev_stair_wave_value)>300)
		{
			stair_wave_value = stair_wave_value - 300;
			prev_stair_wave_value = stair_wave_value;
		}
	}
	else
	{
		prev_stair_wave_value = stair_wave_value;
		stair_wave_value = stair_wave_value - 300;
		if (abs(stair_wave_value-prev_stair_wave_value)>300)
		{
			stair_wave_value = stair_wave_value + 300;
			prev_stair_wave_value = stair_wave_value;
		}
	}
	TIM4->SR &= ~TIM_SR_UIF;
			
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