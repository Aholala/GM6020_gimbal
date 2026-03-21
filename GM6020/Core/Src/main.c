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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "bsp_can.h"
#include "module_pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PRINT_INTERVAL_MS  100   // 打印间隔ms
uint32_t last_print_tick = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//电机PID部分
extern moto_info_t motor_info[MOTOR_MAX_NUM];
pid_struct_t motor_speed_pid[MOTOR_MAX_NUM];
pid_struct_t motor_angle_pid[MOTOR_MAX_NUM];
//float target_speed = 200.0f;//目标速度 串级速度不是恒定的
float target_angle = 0.0f;//目标角度

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
//非阻塞式打印部分
static char printf_buffer[100];//DMA缓冲区
static volatile uint8_t uart_tx_busy = 0;//串口忙标志

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uart_tx_busy = 0;
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  can_user_init(&hcan1);                   // config can filter, start can
  //速度环  输出电压 限幅30000
  pid_init(&motor_speed_pid[0], 0, 0, 0, 10000, 30000);
  //位置环 输出目标转速 限幅为200rpm
  pid_init(&motor_angle_pid[0], 0, 0, 0, 0, 200);
  /*源代码给7个电机初始化
   for (uint8_t i = 0; i < 7; i++)
  {
    pid_init(&motor_pid[i], 40, 3, 0, 3000, 16384);
  }*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //当前角度0-8191转换到0-360
    float current_angle = (float)motor_info[0].rotor_angle * 360.0f /8192.0f;
    //外环位置环 输出目标转速
    float target_speed = pid_calc(&motor_angle_pid[0], target_angle, current_angle);
    //内环速度环 输出电压
    motor_info[0].set_voltage = (int16_t)pid_calc(&motor_speed_pid[0], target_speed, (float)motor_info[0].rotor_speed);
    set_motor_voltage(0,motor_info[0].set_voltage,0,0,0);

    /*原demo控制7电机
     for (uint8_t i = 0; i < 1; i++)
    {
      motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed, motor_info[i].rotor_speed);
    }
    set_motor_voltage(0,
                      motor_info[0].set_voltage,
                      motor_info[1].set_voltage,
                      motor_info[2].set_voltage,
                      motor_info[3].set_voltage);

    set_motor_voltage(1,
                      motor_info[4].set_voltage,
                      motor_info[5].set_voltage,
                      motor_info[6].set_voltage,
                      0);*/

    //每隔 PRINT_INTERVAL_MS 打印一次速度
    uint32_t now = HAL_GetTick();
    if (now - last_print_tick >= PRINT_INTERVAL_MS)
    {
      last_print_tick = now;

      if(!uart_tx_busy) {
        uart_tx_busy = 1;
        int len = snprintf(printf_buffer, sizeof(printf_buffer),
                                   "target_angle:%.1f actual_angle:%.1f actual_speed:%d\r\n",
                                   target_angle,
                                   current_angle,
                                   (int)motor_info[0].rotor_speed);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)printf_buffer, len);
      }
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
