/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Dual GM6020 angle sweep ±30° from power-on zero
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "can.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdarg.h"
#include "bsp_can.h"
#include "module_pid.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PM */
extern moto_info_t motor_info[MOTOR_MAX_NUM];

/* ══════════════════════════════════════════
   PID 参数区  ← 调参只改这里
   ══════════════════════════════════════════ */
// ID2
#define KP_ANGLE1   4.0f
#define KI_ANGLE1   0.0f
#define KD_ANGLE1   0.0f
#define KP_SPEED1   80.0f
#define KI_SPEED1   0.0f
#define KD_SPEED1   0.0f

// ID4
#define KP_ANGLE2   4.0f
#define KI_ANGLE2   0.0f
#define KD_ANGLE2   0.0f
#define KP_SPEED2   80.0f
#define KI_SPEED2   0.0f
#define KD_SPEED2   0.0f

/* 扫描参数 */
#define SCAN_RANGE_DEG      30.0f    // 零点左右各 30°
#define SWITCH_INTERVAL_MS  2000     // 切换间隔 ms

/* PID 输出限幅 */
#define ANGLE_OUT_MAX   320.0f       // 外环输出（rpm）
#define ANGLE_I_MAX     100.0f
#define SPEED_OUT_MAX   25000.0f     // 内环输出（电压值）
#define SPEED_I_MAX     8000.0f

/* ══════════════════════════════════════════
   PID 结构体
   ══════════════════════════════════════════ */
pid_struct_t angle_pid1, speed_pid1;   // ID2
pid_struct_t angle_pid2, speed_pid2;   // ID4

/* ══════════════════════════════════════════
   上电零点（编码器原始值，角度制）
   ══════════════════════════════════════════ */
float angle_offset1 = 0.0f;   // ID2 上电零点
float angle_offset2 = 0.0f;   // ID4 上电零点

/* ══════════════════════════════════════════
   扫描控制
   ══════════════════════════════════════════ */
float  target_angle       = SCAN_RANGE_DEG;  // 当前目标（相对零点，度）
uint32_t last_switch_time = 0;

/* ══════════════════════════════════════════
   Ozone / VOFA+ 调试观测变量
   ══════════════════════════════════════════ */
// ID2
float dbg_raw_angle1     = 0.0f;   // 编码器原始角度（绝对，°）
float dbg_current_angle1 = 0.0f;   // 相对零点角度（°）
float dbg_current_speed1 = 0.0f;   // 转速（rpm）
float dbg_angle_error1   = 0.0f;   // 外环误差
float dbg_target_speed1  = 0.0f;   // 外环输出目标速度
float dbg_voltage1       = 0.0f;   // 内环输出电压

// ID4
float dbg_raw_angle2     = 0.0f;
float dbg_current_angle2 = 0.0f;
float dbg_current_speed2 = 0.0f;
float dbg_angle_error2   = 0.0f;
float dbg_target_speed2  = 0.0f;
float dbg_voltage2       = 0.0f;

/* ══════════════════════════════════════════
   辅助函数
   ══════════════════════════════════════════ */
// 误差归一化到 -180 ~ +180°
static inline float angle_error_normalized(float target, float current)
{
    float err = target - current;
    if      (err >  180.0f) err -= 360.0f;
    else if (err < -180.0f) err += 360.0f;
    return err;
}

// 读编码器 → 绝对角度（°）
static inline float encoder_to_deg(uint16_t raw)
{
    return (float)raw * 360.0f / 8192.0f;
}
/* USER CODE END PM */

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN2_Init();

    /* USER CODE BEGIN 2 */
    can_user_init(&hcan2);

    // ID2
    pid_init(&angle_pid1, KP_ANGLE1, KI_ANGLE1, KD_ANGLE1, ANGLE_I_MAX,  ANGLE_OUT_MAX);
    pid_init(&speed_pid1, KP_SPEED1, KI_SPEED1, KD_SPEED1, SPEED_I_MAX,  SPEED_OUT_MAX);
    // ID4
    pid_init(&angle_pid2, KP_ANGLE2, KI_ANGLE2, KD_ANGLE2, ANGLE_I_MAX,  ANGLE_OUT_MAX);
    pid_init(&speed_pid2, KP_SPEED2, KI_SPEED2, KD_SPEED2, SPEED_I_MAX,  SPEED_OUT_MAX);

    /* ── 等待第一帧 CAN 数据到位后记录零点 ── */
    HAL_Delay(200);   // 等 GM6020 上报一次（通常 <2ms，200ms 绰绰有余）
    angle_offset1 = encoder_to_deg(motor_info[2].rotor_angle);
    angle_offset2 = encoder_to_deg(motor_info[4].rotor_angle);
    last_switch_time = HAL_GetTick();
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        static uint32_t last_pid_tick = 0;
        if (HAL_GetTick() - last_pid_tick < 10) continue;
        last_pid_tick = HAL_GetTick();

        /* ── 1. 切换目标角度（相对零点）──────────────── */
        if (HAL_GetTick() - last_switch_time >= SWITCH_INTERVAL_MS)
        {
            last_switch_time = HAL_GetTick();
            target_angle = (target_angle > 0.0f) ? -SCAN_RANGE_DEG : SCAN_RANGE_DEG;
            // 切换时清积分防过冲
            angle_pid1.i_out = 0.0f;  speed_pid1.i_out = 0.0f;
            angle_pid2.i_out = 0.0f;  speed_pid2.i_out = 0.0f;
        }

        /* ══════════ ID2 ══════════ */
        // 2. 反馈
        dbg_raw_angle1     = encoder_to_deg((uint16_t)motor_info[2].rotor_angle);
        dbg_current_angle1 = angle_error_normalized(dbg_raw_angle1, angle_offset1);
        dbg_current_speed1 = (float)motor_info[2].rotor_speed;

        // 3. 外环：角度 → 目标速度
        dbg_angle_error1   = angle_error_normalized(target_angle, dbg_current_angle1);
        angle_pid1.err[1]  = angle_pid1.err[0];
        angle_pid1.err[0]  = dbg_angle_error1;
        angle_pid1.p_out   = angle_pid1.kp * angle_pid1.err[0];
        angle_pid1.i_out  += angle_pid1.ki * angle_pid1.err[0];
        LIMIT_MIN_MAX(angle_pid1.i_out, -angle_pid1.i_max, angle_pid1.i_max);
        angle_pid1.d_out   = angle_pid1.kd * (angle_pid1.err[0] - angle_pid1.err[1]);
        dbg_target_speed1  = angle_pid1.p_out + angle_pid1.i_out + angle_pid1.d_out;
        LIMIT_MIN_MAX(dbg_target_speed1, -angle_pid1.out_max, angle_pid1.out_max);

        // 4. 内环：速度 → 电压
        float speed_err1   = dbg_target_speed1 - dbg_current_speed1;
        speed_pid1.err[1]  = speed_pid1.err[0];
        speed_pid1.err[0]  = speed_err1;
        speed_pid1.p_out   = speed_pid1.kp * speed_pid1.err[0];
        speed_pid1.i_out  += speed_pid1.ki * speed_pid1.err[0];
        LIMIT_MIN_MAX(speed_pid1.i_out, -speed_pid1.i_max, speed_pid1.i_max);
        speed_pid1.d_out   = speed_pid1.kd * (speed_pid1.err[0] - speed_pid1.err[1]);
        dbg_voltage1       = speed_pid1.p_out + speed_pid1.i_out + speed_pid1.d_out;
        LIMIT_MIN_MAX(dbg_voltage1, -speed_pid1.out_max, speed_pid1.out_max);

        /* ══════════ ID4 ══════════ */
        // 2. 反馈
        dbg_raw_angle2     = encoder_to_deg((uint16_t)motor_info[4].rotor_angle);
        dbg_current_angle2 = angle_error_normalized(dbg_raw_angle2, angle_offset2);
        dbg_current_speed2 = (float)motor_info[4].rotor_speed;

        // 3. 外环：角度 → 目标速度
        dbg_angle_error2   = angle_error_normalized(target_angle, dbg_current_angle2);
        angle_pid2.err[1]  = angle_pid2.err[0];
        angle_pid2.err[0]  = dbg_angle_error2;
        angle_pid2.p_out   = angle_pid2.kp * angle_pid2.err[0];
        angle_pid2.i_out  += angle_pid2.ki * angle_pid2.err[0];
        LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
        angle_pid2.d_out   = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
        dbg_target_speed2  = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;
        LIMIT_MIN_MAX(dbg_target_speed2, -angle_pid2.out_max, angle_pid2.out_max);

        // 4. 内环：速度 → 电压
        float speed_err2   = dbg_target_speed2 - dbg_current_speed2;
        speed_pid2.err[1]  = speed_pid2.err[0];
        speed_pid2.err[0]  = speed_err2;
        speed_pid2.p_out   = speed_pid2.kp * speed_pid2.err[0];
        speed_pid2.i_out  += speed_pid2.ki * speed_pid2.err[0];
        LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);
        speed_pid2.d_out   = speed_pid2.kd * (speed_pid2.err[0] - speed_pid2.err[1]);
        dbg_voltage2       = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;
        LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);

        /* ── 5. 发送 ── */
        set_motor_voltage(0, 0, (int16_t)dbg_voltage1, 0, (int16_t)dbg_voltage2);
        /* USER CODE END 3 */
    }
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