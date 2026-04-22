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
#include "math.h"
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

#define OPEN_LOOP_TEST    0
#define OPEN_LOOP_VOLTAGE 5000

/* ===== 电机索引 =====================================================
 *
 *  ID2：反馈帧 0x206，motor_info[2]，控制帧 0x1FF v2 位置
 *  ID4：反馈帧 0x208，motor_info[4]，控制帧 0x1FF v4 位置
 *  两个电机共用同一个 prof_pos，同步正弦扫描
 *
 * ==================================================================== */
#define MOTOR2_IDX  2
#define MOTOR4_IDX  4

/* ===== 正弦扫描参数 ================================================= */
#define SCAN_AMP          40.0f
#define SCAN_FREQ          1.0f

#define SPEED_FF_GAIN      0.0f

#define HOMING_SPEED      200.0f
#define HOMING_THRESH      3.0f

#define ANGLE_KP           6.0f
#define ANGLE_KI           0.005f
#define ANGLE_KD           0.0f
#define ANGLE_I_MAX       40.0f
#define ANGLE_OUT_MAX    250.0f

#define SPEED_KP         120.0f
#define SPEED_KI           0.05f
#define SPEED_KD           0.0f
#define SPEED_I_MAX      8000.0f
#define SPEED_OUT_MAX   25000.0f

#define LPF_ALPHA          0.3f

// 每个电机独立的 PID 和状态
pid_struct_t angle_pid2,  angle_pid4;
pid_struct_t speed_pid2,  speed_pid4;

typedef enum {
  STATE_HOMING = 0,
  STATE_SCAN
} ctrl_state_t;

// 两个电机各自的归位状态，都到位后才开始扫描
ctrl_state_t state2 = STATE_HOMING;
ctrl_state_t state4 = STATE_HOMING;

// 共用虚拟目标和扫描相位
float prof_pos    = 0.0f;
float scan_phase  = 0.0f;
float dbg_spd_ff  = 0.0f;   // 前馈速度（两个电机共用）

// 各电机独立的低通滤波速度
float filtered_spd2 = 0.0f;
float filtered_spd4 = 0.0f;

// 上电零点
uint16_t zero_enc2 = 0;
uint16_t zero_enc4 = 0;

/* ===== Ozone 观测变量 ==============================================
 *
 *  Data Sampling 加：
 *  dbg_angle2     ID2 当前角度
 *  dbg_angle4     ID4 当前角度
 *  dbg_prof       共用虚拟目标（正弦波 ±30°）
 *  dbg_err2       ID2 跟踪误差
 *  dbg_err4       ID4 跟踪误差
 *  dbg_voltage2   ID2 输出电压
 *  dbg_voltage4   ID4 输出电压
 *  dbg_state      0=归位  1=扫描（两个都到位才变1）
 *
 * ================================================================== */
volatile float dbg_angle2    = 0.0f;
volatile float dbg_angle4    = 0.0f;
volatile float dbg_prof      = 0.0f;
volatile float dbg_err2      = 0.0f;
volatile float dbg_err4      = 0.0f;
volatile float dbg_voltage2  = 0.0f;
volatile float dbg_voltage4  = 0.0f;
volatile int   dbg_state     = 0;

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

// 编码器原始值 → 角度，以各自零点为 0°
float enc_to_angle(uint16_t enc, uint16_t zero)
{
  int32_t diff = (int32_t)enc - (int32_t)zero;
  if      (diff >  4096) diff -= 8192;
  else if (diff < -4096) diff += 8192;
  return (float)diff * 360.0f / 8192.0f;
}

float angle_err_calc(float target, float current)
{
  float e = target - current;
  if      (e >  180.0f) e -= 360.0f;
  else if (e < -180.0f) e += 360.0f;
  return e;
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
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  can_user_init(&hcan2);

  pid_init(&angle_pid2, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_I_MAX, ANGLE_OUT_MAX);
  pid_init(&speed_pid2, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_I_MAX, SPEED_OUT_MAX);
  pid_init(&angle_pid4, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_I_MAX, ANGLE_OUT_MAX);
  pid_init(&speed_pid4, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_I_MAX, SPEED_OUT_MAX);

  // 等第一帧 CAN 数据
  HAL_Delay(100);

  // 记录两个电机的上电零点
  zero_enc2     = motor_info[MOTOR2_IDX].rotor_angle;
  zero_enc4     = motor_info[MOTOR4_IDX].rotor_angle;
  prof_pos      = 0.0f;
  filtered_spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
  filtered_spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick >= 10)
    {
      last_tick = HAL_GetTick();

      const float dt = 0.01f;

#if OPEN_LOOP_TEST
      set_motor_voltage(0, 0, (int16_t)OPEN_LOOP_VOLTAGE,
                              0,
                              (int16_t)OPEN_LOOP_VOLTAGE);

#else
      // 读取两个电机反馈
      dbg_angle2 = enc_to_angle(motor_info[MOTOR2_IDX].rotor_angle, zero_enc2);
      dbg_angle4 = enc_to_angle(motor_info[MOTOR4_IDX].rotor_angle, zero_enc4);

      float spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
      float spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;

      filtered_spd2 = LPF_ALPHA * spd2 + (1.0f - LPF_ALPHA) * filtered_spd2;
      filtered_spd4 = LPF_ALPHA * spd4 + (1.0f - LPF_ALPHA) * filtered_spd4;

      // 归位阶段：两个电机都归位到 0° 才开始扫描
      if (state2 == STATE_HOMING)
      {
        float step = HOMING_SPEED * dt;
        float diff = angle_err_calc(0.0f, prof_pos);
        if      (diff >  step) prof_pos += step;
        else if (diff < -step) prof_pos -= step;
        else                   prof_pos  = 0.0f;

        if (fabsf(angle_err_calc(0.0f, dbg_angle2)) < HOMING_THRESH)
          state2 = STATE_SCAN;
      }

      if (state4 == STATE_HOMING)
      {
        if (fabsf(angle_err_calc(0.0f, dbg_angle4)) < HOMING_THRESH)
          state4 = STATE_SCAN;
      }

      // 两个都归位完成后才开始扫描
      int both_scanning = (state2 == STATE_SCAN && state4 == STATE_SCAN);
      dbg_state = both_scanning;

      if (both_scanning)
      {
        scan_phase += dt;
        float omega    = 2.0f * (float)M_PI * SCAN_FREQ;
        prof_pos       = SCAN_AMP * sinf(omega * scan_phase);
        float ff_deg_s = SCAN_AMP * omega * cosf(omega * scan_phase);
        dbg_spd_ff     = ff_deg_s * SPEED_FF_GAIN;
      }

      dbg_prof = prof_pos;

      // ---- ID2 PID ----
      {
        dbg_err2             = angle_err_calc(prof_pos, dbg_angle2);
        angle_pid2.err[1]    = angle_pid2.err[0];
        angle_pid2.err[0]    = dbg_err2;
        angle_pid2.p_out     = angle_pid2.kp * angle_pid2.err[0];
        angle_pid2.i_out    += angle_pid2.ki * angle_pid2.err[0];
        LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
        angle_pid2.d_out     = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
        float pid_spd2       = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;

        float spd_target2    = pid_spd2 + dbg_spd_ff;
        LIMIT_MIN_MAX(spd_target2, -angle_pid2.out_max, angle_pid2.out_max);

        float spd_err2       = spd_target2 - filtered_spd2;
        speed_pid2.err[1]    = speed_pid2.err[0];
        speed_pid2.err[0]    = spd_err2;
        speed_pid2.p_out     = speed_pid2.kp * speed_pid2.err[0];
        speed_pid2.i_out    += speed_pid2.ki * speed_pid2.err[0];
        LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);
        speed_pid2.d_out     = speed_pid2.kd * (speed_pid2.err[0] - speed_pid2.err[1]);
        dbg_voltage2         = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;
        LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);
      }

      // ---- ID4 PID ----
      {
        dbg_err4             = angle_err_calc(prof_pos, dbg_angle4);
        angle_pid4.err[1]    = angle_pid4.err[0];
        angle_pid4.err[0]    = dbg_err4;
        angle_pid4.p_out     = angle_pid4.kp * angle_pid4.err[0];
        angle_pid4.i_out    += angle_pid4.ki * angle_pid4.err[0];
        LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);
        angle_pid4.d_out     = angle_pid4.kd * (angle_pid4.err[0] - angle_pid4.err[1]);
        float pid_spd4       = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;

        float spd_target4    = pid_spd4 + dbg_spd_ff;
        LIMIT_MIN_MAX(spd_target4, -angle_pid4.out_max, angle_pid4.out_max);

        float spd_err4       = spd_target4 - filtered_spd4;
        speed_pid4.err[1]    = speed_pid4.err[0];
        speed_pid4.err[0]    = spd_err4;
        speed_pid4.p_out     = speed_pid4.kp * speed_pid4.err[0];
        speed_pid4.i_out    += speed_pid4.ki * speed_pid4.err[0];
        LIMIT_MIN_MAX(speed_pid4.i_out, -speed_pid4.i_max, speed_pid4.i_max);
        speed_pid4.d_out     = speed_pid4.kd * (speed_pid4.err[0] - speed_pid4.err[1]);
        dbg_voltage4         = speed_pid4.p_out + speed_pid4.i_out + speed_pid4.d_out;
        LIMIT_MIN_MAX(dbg_voltage4, -speed_pid4.out_max, speed_pid4.out_max);
      }

      // 一帧同时发送两个电机的电压
      // 0x1FF：v1=ID1, v2=ID2, v3=ID3, v4=ID4
      set_motor_voltage(0,
                        0,
                        (int16_t)dbg_voltage2,
                        0,
                        (int16_t)dbg_voltage4);

#endif
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
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
