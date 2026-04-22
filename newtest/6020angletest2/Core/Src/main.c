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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "stdarg.h"
#include "bsp_can.h"
#include "module_pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern moto_info_t motor_info[MOTOR_MAX_NUM];
//============ID2============
pid_struct_t angle_pid1;   // 外环（角度）
pid_struct_t speed_pid1;   // 内环（速度）
//============ID4============
pid_struct_t angle_pid2;
pid_struct_t speed_pid2;

// 角度环控制变量
float target_angle = 30.0f;               // 目标角度（度）
uint32_t last_switch_time = 0;
const uint32_t SWITCH_INTERVAL_MS = 2000; // 每2秒自动切换目标角度

// 调试观测变量（全局，Ozone Watch / Data Sampling 可见）
//============ID2============
float dbg_current_angle1 = 0.0f;   // 当前角度（度）
float dbg_current_speed1 = 0.0f;   // 当前转速（rpm）
float dbg_angle_error1   = 0.0f;   // 角度误差（度）
float dbg_target_speed1  = 0.0f;   // 外环输出目标速度（rpm）
float dbg_voltage1       = 0.0f;   // 内环输出电压

//============ID4============
float dbg_current_angle2 = 0.0f;   // 当前角度（度）
float dbg_current_speed2 = 0.0f;   // 当前转速（rpm）
float dbg_angle_error2   = 0.0f;   // 角度误差（度）
float dbg_target_speed2  = 0.0f;   // 外环输出目标速度（rpm）
float dbg_voltage2       = 0.0f;   // 内环输出电压

// 角度误差归一化（保证误差在 -180°~+180° 之间）
float angle_error_normalized(float target, float current)
{
  float err = target - current;
  if (err > 180.0f) err -= 360.0f;
  else if (err < -180.0f) err += 360.0f;
  return err;
}

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
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  can_user_init(&hcan2);   // config can filter, start can
  //============ID2============
  // 外环角度环
  pid_init(&angle_pid1, 4.0f, 0.0f, 0.0f, 100.0f, 320.0f);
  // 内环速度环
  pid_init(&speed_pid1, 80.0f, 0.0f, 0.0f, 8000.0f, 25000.0f);


  //============ID4============
  // 外环角度环
  pid_init(&angle_pid2, 4.0f, 0.0f, 0.0f, 100.0f, 320.0f);
  // 内环速度环
  pid_init(&speed_pid2, 80.0f, 0.0f, 0.0f, 8000.0f, 25000.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_pid_tick = 0;
    if (HAL_GetTick() - last_pid_tick >= 10)
    {
      last_pid_tick = HAL_GetTick();

      // ----- 1. 每2秒自动切换目标角度（+60° ↔ -60°）-----
      if (HAL_GetTick() - last_switch_time >= SWITCH_INTERVAL_MS)
      {
        last_switch_time = HAL_GetTick();
        target_angle = (target_angle > 0) ? -30.0f : 30.0f;
        // 切换时清积分，防止过冲
        angle_pid1.i_out = 0.0f;
        speed_pid1.i_out = 0.0f;
        angle_pid2.i_out = 0.0f;
        speed_pid2.i_out = 0.0f;
      }


      //============ID2============
      // ----- 2. 获取反馈 -----
      // GM6020 编码器 0~8191 对应 0~360°
      // ID2：反馈帧 0x206，index = 2
      dbg_current_angle1 = (float)motor_info[2].rotor_angle * 360.0f / 8192.0f;
      dbg_current_speed1 = (float)motor_info[2].rotor_speed;

      // ----- 3. 外环：角度 → 目标速度 -----
      dbg_angle_error1 = angle_error_normalized(target_angle, dbg_current_angle1);
      angle_pid1.err[1] = angle_pid1.err[0];
      angle_pid1.err[0] = dbg_angle_error1;
      angle_pid1.p_out  = angle_pid1.kp * angle_pid1.err[0];
      angle_pid1.i_out += angle_pid1.ki * angle_pid1.err[0];
      LIMIT_MIN_MAX(angle_pid1.i_out, -angle_pid1.i_max, angle_pid1.i_max);
      angle_pid1.d_out  = angle_pid1.kd * (angle_pid1.err[0] - angle_pid1.err[1]);
      dbg_target_speed1 = angle_pid1.p_out + angle_pid1.i_out + angle_pid1.d_out;
      LIMIT_MIN_MAX(dbg_target_speed1, -angle_pid1.out_max, angle_pid1.out_max);

      // ----- 4. 内环：目标速度 → 电压 -----
      float speed_error = dbg_target_speed1 - dbg_current_speed1;
      speed_pid1.err[1] = speed_pid1.err[0];
      speed_pid1.err[0] = speed_error;
      speed_pid1.p_out  = speed_pid1.kp * speed_pid1.err[0];
      speed_pid1.i_out += speed_pid1.ki * speed_pid1.err[0];
      LIMIT_MIN_MAX(speed_pid1.i_out, -speed_pid1.i_max, speed_pid1.i_max);
      speed_pid1.d_out  = speed_pid1.kd * (speed_pid1.err[0] - speed_pid1.err[1]);
      dbg_voltage1 = speed_pid1.p_out + speed_pid1.i_out + speed_pid1.d_out;
      LIMIT_MIN_MAX(dbg_voltage1, -speed_pid1.out_max, speed_pid1.out_max);


      //============ID4============
      // ----- 2. 获取反馈 -----
      dbg_current_angle2 = (float)motor_info[4].rotor_angle * 360.0f / 8192.0f;
      dbg_current_speed2 = (float)motor_info[4].rotor_speed;

      // ----- 3. 外环：角度 → 目标速度 -----
      dbg_angle_error2     = angle_error_normalized(target_angle, dbg_current_angle2);
      angle_pid2.err[1]    = angle_pid2.err[0];
      angle_pid2.err[0]    = dbg_angle_error2;
      angle_pid2.p_out     = angle_pid2.kp * angle_pid2.err[0];
      angle_pid2.i_out    += angle_pid2.ki * angle_pid2.err[0];
      LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
      angle_pid2.d_out     = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
      dbg_target_speed2    = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;
      LIMIT_MIN_MAX(dbg_target_speed2, -angle_pid2.out_max, angle_pid2.out_max);

      // ----- 4. 内环：目标速度 → 电压 -----
      float speed_error2   = dbg_target_speed2 - dbg_current_speed2;
      speed_pid2.err[1]    = speed_pid2.err[0];
      speed_pid2.err[0]    = speed_error2;
      speed_pid2.p_out     = speed_pid2.kp * speed_pid2.err[0];
      speed_pid2.i_out    += speed_pid2.ki * speed_pid2.err[0];
      LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);
      speed_pid2.d_out     = speed_pid2.kd * (speed_pid2.err[0] - speed_pid2.err[1]);
      dbg_voltage2         = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;
      LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);

      // ----- 5. 发送电压给电机 -----
      set_motor_voltage(0, 0, (int16_t)dbg_voltage1, 0, (int16_t)dbg_voltage2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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