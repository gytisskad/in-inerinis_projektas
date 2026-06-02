/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "i2c-lcd.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Matavimai ir UART duomenų siuntimas atliekami kas 200 ms,
   nes projekte reikalaujamas 0,2 s diskretizavimo periodas. */
#define SAMPLE_PERIOD_MS   200 
/* LCD ekranas atnaujinamas kas 2 s pagal užduoties reikalavimą. */
#define LCD_PERIOD_MS      2000
/* Naudojamas 5 paskutinių matavimų slankusis vidurkis,
   kad sumažėtų atsitiktiniai HC-SR04 rodmenų svyravimai. */
#define FILTER_SIZE        5

#define MIN_DISTANCE_X10   10      // 1.0 cm
#define MAX_DISTANCE_X10   300     // 30.0 cm

int16_t ch1_buffer[FILTER_SIZE] = {0};
int16_t ch2_buffer[FILTER_SIZE] = {0};

uint8_t filter_index = 0;
uint8_t filter_count = 0;

int16_t d1_avg_x10 = 0;
int16_t d2_avg_x10 = 0;
int16_t vid_x10 = 0;
int16_t skirt_x10 = 0;

uint32_t paskutinis_matavimas = 0;
uint32_t last_lcd_time = 0;

char uart_msg[120];
char lcd_line1[17];
char lcd_line2[17];

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}
/* HC-SR04 matavimas pradedamas 10 us TRIG impulsu. */
void HCSR04_Trigger(GPIO_TypeDef* trig_port, uint16_t trig_pin)
{
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    delay_us(3);

    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    delay_us(10);

    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
}
/* Matuojama ECHO impulso trukmė. Ji atitinka ultragarso
   signalo keliavimo iki objekto ir atgal laiką. */
uint32_t HCSR04_ReadEcho(GPIO_TypeDef* echo_port, uint16_t echo_pin)
{
    uint32_t timeout;

    timeout = 30000;

    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET)
    {
        if (timeout-- == 0)
        {
            return 0;
        }
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0);

    timeout = 30000;

    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
    {
        if (timeout-- == 0)
        {
            return 0;
        }
    }

    return __HAL_TIM_GET_COUNTER(&htim2);
}

int16_t HCSR04_GetDistanceX10(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                              GPIO_TypeDef* echo_port, uint16_t echo_pin)
{
    HCSR04_Trigger(trig_port, trig_pin);

    uint32_t echo_time_us = HCSR04_ReadEcho(echo_port, echo_pin);

    if (echo_time_us == 0)
    {
        return -1;
    }

    /*
       distance_cm = time_us * 0.0343 / 2
       distance_x10 = distance_cm * 10
       distance_x10 ~ time_us * 1715 / 10000
    */
		
		/* Atstumas apskaičiuojamas pagal formulę:
   d = v * t / 2, kur v ≈ 343 m/s.
   Reikšmė saugoma dešimtosiomis cm dalimis. */
    int16_t distance_x10 = (int16_t)((echo_time_us * 1715UL) / 10000UL);

    return distance_x10;
}

int16_t AverageBuffer(int16_t* buffer, uint8_t count)
{
    int32_t sum = 0;

    if (count == 0)
    {
        return 0;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        sum += buffer[i];
    }

    return (int16_t)(sum / count);
}
/* Tikrinama, ar matavimo rezultatas patenka į numatytą 1–30 cm diapazoną. */
uint8_t DistanceValid(int16_t d_x10)
{
    if (d_x10 >= MIN_DISTANCE_X10 && d_x10 <= MAX_DISTANCE_X10)
    {
        return 1;
    }

    return 0;
}
/* Skaičius paverčiamas tekstu su viena dešimtaine dalimi */
void FormatDistance(char* text, int16_t value_x10)
{
    int16_t whole = value_x10 / 10;
    int16_t decimal = value_x10 % 10;

    if (decimal < 0)
    {
        decimal = -decimal;
    }

    sprintf(text, "%d.%d", whole, decimal);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

HAL_TIM_Base_Start(&htim2);

lcd_init();
lcd_clear();
lcd_put_cur(0, 0);
lcd_send_string("Atstumo mat.");
lcd_put_cur(1, 0);
lcd_send_string("Sistema OK");
HAL_Delay(1000);
lcd_clear();

char start_msg[] = "Atstumo matavimo sistema paleista\r\n";
HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		 uint32_t now = HAL_GetTick();

    if (now - paskutinis_matavimas >= SAMPLE_PERIOD_MS)
    {
        paskutinis_matavimas = now;

        int16_t ch1_raw_x10;
        int16_t ch2_raw_x10;

        char status[20];

        ch1_raw_x10 = HCSR04_GetDistanceX10(TRIG1_GPIO_Port,
                                            TRIG1_Pin,
                                            ECHO1_GPIO_Port,
                                            ECHO1_Pin);

        HAL_Delay(60);

        ch2_raw_x10 = HCSR04_GetDistanceX10(TRIG2_GPIO_Port,
                                            TRIG2_Pin,
                                            ECHO2_GPIO_Port,
                                            ECHO2_Pin);

        HAL_Delay(60);

        if (ch1_raw_x10 < 0 || ch2_raw_x10 < 0)
        {
            strcpy(status, "Klaida");
        }
        else if (!DistanceValid(ch1_raw_x10) || !DistanceValid(ch2_raw_x10))
        {
            strcpy(status, "UZ_RIBU");
        }
        else
        {
            strcpy(status, "OK");

            ch1_buffer[filter_index] = ch1_raw_x10;
            ch2_buffer[filter_index] = ch2_raw_x10;

            filter_index++;

            if (filter_index >= FILTER_SIZE)
            {
                filter_index = 0;
            }

            if (filter_count < FILTER_SIZE)
            {
                filter_count++;
            }

            d1_avg_x10 = AverageBuffer(ch1_buffer, filter_count);
            d2_avg_x10 = AverageBuffer(ch2_buffer, filter_count);

            vid_x10 = (d1_avg_x10 + d2_avg_x10) / 2;
            skirt_x10 = d1_avg_x10 - d2_avg_x10;
        }

        char d1_text[10];
        char d2_text[10];
        char vid_text[10];
        char skirt_text[10];

        FormatDistance(d1_text, d1_avg_x10);
        FormatDistance(d2_text, d2_avg_x10);
        FormatDistance(vid_text, vid_x10);
        FormatDistance(skirt_text, skirt_x10);

        snprintf(uart_msg, sizeof(uart_msg),
                 "D1=%s cm  D2=%s cm  VID=%s cm  SKIRT=%s cm  BUSENA=%s\r\n",
                 
                 d1_text,
                 d2_text,
                 vid_text,
                 skirt_text,
                 status);

        HAL_UART_Transmit(&huart2,
                          (uint8_t*)uart_msg,
                          strlen(uart_msg),
                          HAL_MAX_DELAY);
				
/* Į LCD kas 2 s išvedami abiejų kanalų atstumai,
   jų vidurkis ir skirtumas. */
if (now - last_lcd_time >= LCD_PERIOD_MS)
{
    last_lcd_time = now;

    lcd_clear();

    lcd_put_cur(0, 0);
    snprintf(lcd_line1, sizeof(lcd_line1),
             "D1:%s D2:%s",
             d1_text,
             d2_text);
    lcd_send_string(lcd_line1);

    lcd_put_cur(1, 0);
    snprintf(lcd_line2, sizeof(lcd_line2),
             "V:%s S:%s",
             vid_text,
             skirt_text);
    lcd_send_string(lcd_line2);
}
				
    /* USER CODE END WHILE */
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00805C87;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG1_Pin|TRIG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG1_Pin TRIG2_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin|TRIG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO1_Pin ECHO2_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
