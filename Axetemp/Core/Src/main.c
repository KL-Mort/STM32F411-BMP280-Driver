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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "BMP280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_ADDR (0x77 << 1)
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
static BMP280 bmp280_sensor;

BMP280_Config conf={
    		.mode	= BMP280_MODE_SLEEP,
			.osrs_t	= BMP280_OSRS_2X,
			.osrs_p	= BMP280_OSRS_16X,
			.filter	= BMP280_FILTER_4,
    		.standby= BMP280_STANDBY_0_5_MS
    };

static bool led_on = false;
static uint32_t led_off_timestamp = 0;
static uint8_t rx_byte = 0;

static volatile bool timer_flag = false;
static volatile bool samp_activ = false;
static volatile bool read_f = false;
static volatile bool mood_flag = false;
static volatile bool conf_f = false;

static volatile uint8_t rx_cmd_t = 0;
static volatile uint8_t rx_state = 0;

static const uint32_t inter_arr [4]={4999, 9999, 49999, 99999};
static const float inter_sec [4]={0.5f, 1.0f, 5.0f, 10.0f};
static uint8_t inter_idx = 1;

static const BMP280_Filter fil_list[5]={
		BMP280_FILTER_OFF, BMP280_FILTER_2, BMP280_FILTER_4, BMP280_FILTER_8, BMP280_FILTER_16
};
static const char *fil_nam[5] = {"OFF", "x2", "x4", "x8", "x16"};
static uint8_t fil_idx = 2;

static const BMP280_Oversampling osrs_list [5] ={
		BMP280_OSRS_1X, BMP280_OSRS_2X, BMP280_OSRS_4X, BMP280_OSRS_8X, BMP280_OSRS_16X
};

static const char *osrs_nam[5] = {"x1", "x2", "x4", "x8", "x16"};
static uint8_t osrs_idx = 1;

static int32_t I2C_Read_Adapter(void *context, uint8_t reg,
		uint8_t *data, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)context;
	return (HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDR_VCC, reg,
			I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK) ? 0 : -1;
}

static int32_t I2C_Write_Adapter(void *context, uint8_t reg,
		const uint8_t *data, uint16_t len)

{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)context;
	return (HAL_I2C_Mem_Write(hi2c, BMP280_I2C_ADDR_VCC, reg,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, 100)== HAL_OK)? 0 : -1;

}

static void print_uart(const char *str)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

static void read_send (void)
{
	float temp = 0.0f;
	float press = 0.0f;
		if (BMP280_ReadData (&bmp280_sensor, &temp, &press) == BMP280_OK)
		{

			char uart_buf[64];
			snprintf(uart_buf, sizeof(uart_buf),
					"T=%.1fC P=%.1fhPa\r\n", temp, press);
			print_uart(uart_buf);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			led_off_timestamp = HAL_GetTick();
			led_on = true;
		}
		else
		{

			print_uart("[ERROR] Sensor read failed!!1\r\n");
		}

}

static void read_send_fors(void)
{
		float temp = 0.0f;
		float press = 0.0f;
	if (BMP280_ReadDataForce(&bmp280_sensor, &temp, &press) == BMP280_OK)
			{

				char uart_buf[64];
				snprintf(uart_buf, sizeof(uart_buf),
						"T=%.1fC P=%.1fhPa\r\n", temp, press);
				print_uart(uart_buf);

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				led_off_timestamp = HAL_GetTick();
				led_on = true;
			}
			else
			{

				print_uart("[ERROR] Sensor read failed!!1\r\n");
			}

	if (samp_activ)
	{
		BMP280_SetMode(&bmp280_sensor, BMP280_MODE_NORMAL);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		timer_flag = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Обробка цифри після I, O або F
        if (rx_state != 0)
        {
            uint8_t state = rx_state;
            rx_state = 0;

            if (state == 1 && rx_byte >= '0' && rx_byte <= '3')
            {
                inter_idx = rx_byte - '0';

                __HAL_TIM_SET_AUTORELOAD(
                    &htim2, inter_arr[inter_idx]);
                __HAL_TIM_SET_COUNTER(&htim2, 0);

                char msg[64];
                snprintf(msg, sizeof(msg),
                         "[CMD] Interval set to %.1f sec\r\n",
                         (double)inter_sec[inter_idx]);
                print_uart(msg);
            }
            else if (state == 2 && rx_byte >= '0' && rx_byte <= '4')
            {
                osrs_idx = rx_byte - '0';

                conf.osrs_t = osrs_list[osrs_idx];
                conf.osrs_p = osrs_list[osrs_idx];
                conf_f = true;

                char msg[64];
                snprintf(msg, sizeof(msg),
                         "[CMD] Oversampling set to %s\r\n",
                         osrs_nam[osrs_idx]);
                print_uart(msg);
            }
            else if (state == 3 && rx_byte >= '0' && rx_byte <= '4')
            {
                fil_idx = rx_byte - '0';

                conf.filter = fil_list[fil_idx];
                conf_f = true;

                char msg[64];
                snprintf(msg, sizeof(msg),
                         "[CMD] IIR filter set to %s\r\n",
                         fil_nam[fil_idx]);
                print_uart(msg);
            }
            else
            {
                print_uart("[ERROR] Invalid parameter\r\n");
            }
        }
        else
        {
            switch (rx_byte)
            {
                case 's':
                case 'S':
                    samp_activ = !samp_activ;
                    mood_flag = true;

                    if (samp_activ)
                        print_uart("[CMD] Auto-sampling STARTED\r\n");
                    else
                        print_uart("[CMD] Auto-sampling STOPPED\r\n");
                    break;

                case 'r':
                case 'R':
                    print_uart("[CMD] Single read requested\r\n");
                    read_f = true;
                    break;

                case 'h':
                case 'H':
                {
                    char info_buf[140];
                    snprintf(info_buf, sizeof(info_buf),
                    		"[INFO] Status: %s | Mode: %s | Filter: %s | OSRS: %s\r\n",
							samp_activ ? "RUNNING" : "STOPPED",
							samp_activ ? "NORMAL" : "SLEEP",
							fil_nam[fil_idx], osrs_nam[osrs_idx]);
                    print_uart(info_buf);
                    break;
                }

                case 'i':
                case 'I':
                    rx_state = 1;
                    break;

                case 'o':
                case 'O':
                    rx_state = 2;
                    break;

                case 'f':
                case 'F':
                    rx_state = 3;
                    break;

                default:
                    print_uart("[ERROR] Unknown command\r\n");
                    break;
            }
        }

        // Обов'язково повторно вмикаємо приймання UART
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
    HAL_Delay(250);

    bmp280_sensor.bus.context	= &hi2c1;
    bmp280_sensor.bus.read		= I2C_Read_Adapter;
    bmp280_sensor.bus.write		= I2C_Write_Adapter;

    if(BMP280_Init(&bmp280_sensor)!=BMP280_OK){
    print_uart("[FAIL]BMP Init failed! Check writing. \r \n");
    	while(1)
    		{
    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    			HAL_Delay(100);
    	}

    }

    print_uart("[OK] Sensor initialized via modular driver! \r \n");

    BMP280_SetConfig(&bmp280_sensor, &conf);
    HAL_Delay (100);


    print_uart("[OK] Timer started, non-blocking mode active\r\n");

    HAL_UART_Receive_IT(&huart1, &rx_byte,1);
    	print_uart("[OK] UART Command Receiver Ready \r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (led_on && (HAL_GetTick() - led_off_timestamp >= 50))
	        {
	            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	            led_on = false;
	        }

	  	  	if (conf_f)
	  	  	{
	  	  		conf_f = false;
	  	  		conf.mode = samp_activ ? BMP280_MODE_NORMAL : BMP280_MODE_SLEEP;
	  	  		BMP280_SetConfig(&bmp280_sensor, &conf);
	  	  	}
	  	  if (mood_flag)
	  	  {
	  	      mood_flag = false;

	  	      if (samp_activ)
	  	      {
	  	          BMP280_SetMode(&bmp280_sensor, BMP280_MODE_NORMAL);
	  	          HAL_TIM_Base_Start_IT(&htim2);
	  	      }
	  	      else
	  	      {
	  	          BMP280_SetMode(&bmp280_sensor, BMP280_MODE_SLEEP);
	  	          HAL_TIM_Base_Stop_IT(&htim2);
	  	      }
	  	  }
	        if (timer_flag == true)
	        {
	        	timer_flag = false;
	          	read_send();
	        }


	        if (read_f == true)
	        {
	        	read_f = false;
	        	read_send_fors();
	        }

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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
