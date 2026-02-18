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
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
#define DIG1_GPIO_Port GPIOC
#define DIG1_Pin       GPIO_PIN_0
#define DIG2_GPIO_Port GPIOC
#define DIG2_Pin       GPIO_PIN_1
#define SEG_A_PIN      GPIO_PIN_0
#define SEG_A_PORT     GPIOB
#define SEG_B_PIN      GPIO_PIN_10
#define SEG_B_PORT     GPIOC
#define SEG_C_PIN      GPIO_PIN_2
#define SEG_C_PORT     GPIOB
#define SEG_D_PIN      GPIO_PIN_3
#define SEG_D_PORT     GPIOB
#define SEG_E_PIN      GPIO_PIN_4
#define SEG_E_PORT     GPIOB
#define SEG_F_PIN      GPIO_PIN_5
#define SEG_F_PORT     GPIOB
#define SEG_G_PIN      GPIO_PIN_6
#define SEG_G_PORT     GPIOB

const uint8_t seg_digits[10] = {
	    0b00111111, // 0
	    0b00000110, // 1
	    0b01011011, // 2
	    0b01001111, // 3
	    0b01100110, // 4
	    0b01101101, // 5
	    0b01111101, // 6
	    0b00000111, // 7
	    0b01111111, // 8
	    0b01101111  // 9
};

uint32_t echo_start_time = 0;
uint32_t echo_end_time  = 0;
uint8_t waiting_for_falling_edge = 0;
uint32_t distance_cm = 0;

#define TRIG_PIN        GPIO_PIN_10
#define TRIG_PORT       GPIOA
#define LED_RED_PIN     GPIO_PIN_8
#define LED_RED_PORT    GPIOA
#define LED_GREEN_PIN   GPIO_PIN_9
#define LED_GREEN_PORT  GPIOA
#define DISTANCE_FILTER_SIZE 5
uint32_t distance_history[DISTANCE_FILTER_SIZE] = {0};
uint8_t distance_history_index = 0;
uint32_t calculate_average_distance(uint32_t new_distance);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DisplayDigit(uint8_t digit, uint8_t position) {
    HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_SET);

    if (position == 0){
    	HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);
    }
    else{
    	HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, (seg_digits[digit] & 0b0000001) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, (seg_digits[digit] & 0b0000010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, (seg_digits[digit] & 0b0000100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, (seg_digits[digit] & 0b0001000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, (seg_digits[digit] & 0b0010000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, (seg_digits[digit] & 0b0100000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, (seg_digits[digit] & 0b1000000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void HCSR04_Read(void)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	HCSR04_Read();
    	HAL_Delay(60);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (waiting_for_falling_edge == 0)
        {
            echo_start_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            waiting_for_falling_edge = 1;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            echo_end_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);

            uint32_t pulse_width;

            if (echo_end_time >= echo_start_time)
            {
                pulse_width = echo_end_time - echo_start_time;
            } else {
                pulse_width = (0xFFFF - echo_start_time + 1) + echo_end_time;
            }

            distance_cm = pulse_width / 58;

            waiting_for_falling_edge = 0;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint8_t current_digit = 0;

    if (htim->Instance == TIM4) {
    	uint32_t avg_distance = calculate_average_distance(distance_cm);
    	uint8_t tens = (avg_distance / 10) % 10;
    	uint8_t ones = avg_distance % 10;

        if (current_digit == 0) {
            DisplayDigit(tens, 0);
            current_digit = 1;
        } else {
            DisplayDigit(ones, 1);
            current_digit = 0;
        }
    }

    if (htim->Instance == TIM3) {
        if (distance_cm < 10) {
            HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
            HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN);
            HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
        }
    }
}

uint32_t calculate_average_distance(uint32_t new_distance) {
    distance_history[distance_history_index] = new_distance;
    distance_history_index = (distance_history_index + 1) % DISTANCE_FILTER_SIZE;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < DISTANCE_FILTER_SIZE; i++) {
        sum += distance_history[i];
    }
    return sum / DISTANCE_FILTER_SIZE;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
    /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
