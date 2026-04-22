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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "stdarg.h"
#include "math.h"
#include "bsp_can.h"
#include "module_pid.h"
#include "../../User/Module/bmi088/BMI088driver.h"
#include "MahonyAHRS.h"

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

/* ===== IMU 参数 =====================================================
 *
 *  GYRO_CALIB_MS      上电静止校准时长（ms），期间采集陀螺零偏均值
 *  ACCEL_VALID_MIN/MAX 加速度幅值门限（m/s²），超出范围说明有线性加速度，
 *                      此时跳过加速度纠正，避免把振动/冲击当成重力方向
 *
 * ==================================================================== */
#define GYRO_CALIB_MS      2000       // 上电静止校准时长 2 秒 = 200 次采样
#define ACCEL_VALID_MIN    8.0f       // 有效加速度下限 m/s²（约 0.82g）
#define ACCEL_VALID_MAX    11.0f      // 有效加速度上限 m/s²（约 1.12g）

/* ===== 正弦扫描参数 ================================================= */
#define SCAN_AMP          40.0f
#define SCAN_FREQ          0.1f

#define SPEED_FF_GAIN      0.77f

#define HOMING_SPEED      20.0f
#define HOMING_THRESH      3.0f

#define ANGLE_KP           6.0f
#define ANGLE_KI           0.05f
#define ANGLE_KD           0.4f
#define ANGLE_I_MAX       40.0f
#define ANGLE_OUT_MAX    250.0f

#define SPEED_KP         100.0f
#define SPEED_KI           0.5f
#define SPEED_KD           0.3f
#define SPEED_I_MAX      3000.0f
#define SPEED_OUT_MAX   25000.0f

#define LPF_ALPHA          0.8f

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
float dbg_spd_ff  = 0.0f;

// 各电机独立的低通滤波速度
float filtered_spd2 = 0.0f;
float filtered_spd4 = 0.0f;

// 上电零点
uint16_t zero_enc2 = 0;
uint16_t zero_enc4 = 0;

/* ===== Ozone 观测变量 ==============================================
 *
 *  电机相关：
 *  dbg_angle2 / dbg_angle4      当前角度（°）
 *  dbg_prof                     共用虚拟目标
 *  dbg_err2 / dbg_err4          跟踪误差
 *  dbg_voltage2 / dbg_voltage4  输出电压
 *  dbg_state                    0=归位  1=扫描
 *
 *  BMI088 原始数据：
 *  dbg_gyro_x/y/z               陀螺仪 rad/s
 *  dbg_accel_x/y/z              加速度计 m/s²
 *  dbg_temp                     IMU 温度 °C
 *  dbg_imu_error                0=初始化正常
 *
 *  Mahony 姿态融合输出（单位：度）：
 *  dbg_roll                     横滚角
 *  dbg_pitch                    俯仰角
 *  dbg_yaw                      航向角（无磁力计会缓慢漂移）
 *
 * ================================================================== */
volatile float  dbg_angle2      = 0.0f;
volatile float  dbg_angle4      = 0.0f;
volatile float  dbg_prof        = 0.0f;
volatile float  dbg_err2        = 0.0f;
volatile float  dbg_err4        = 0.0f;
volatile float  dbg_voltage2    = 0.0f;
volatile float  dbg_voltage4    = 0.0f;
volatile int    dbg_state       = 0;

volatile fp32   dbg_gyro_x      = 0.0f;
volatile fp32   dbg_gyro_y      = 0.0f;
volatile fp32   dbg_gyro_z      = 0.0f;
volatile fp32   dbg_accel_x     = 0.0f;
volatile fp32   dbg_accel_y     = 0.0f;
volatile fp32   dbg_accel_z     = 0.0f;
volatile fp32   dbg_temp        = 0.0f;
volatile uint8_t dbg_imu_error  = 0;

volatile float  dbg_roll        = 0.0f;
volatile float  dbg_pitch       = 0.0f;
volatile float  dbg_yaw         = 0.0f;

// 陀螺仪零偏（上电静止校准结果）
float gyro_bias[3]      = {0.0f, 0.0f, 0.0f};
// 欧拉角安装偏移（校准完成后第一帧姿态，之后输出时减掉，使上电即为零点）
float euler_offset[3]   = {0.0f, 0.0f, 0.0f};  // [roll, pitch, yaw]
// 校准状态：0=校准中，1=校准完成
volatile int imu_calib_done     = 0;
// Ozone 可观测：校准期间累计样本数
volatile int dbg_calib_samples  = 0;

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

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  can_user_init(&hcan2);

  pid_init(&angle_pid2, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_I_MAX, ANGLE_OUT_MAX);
  pid_init(&speed_pid2, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_I_MAX, SPEED_OUT_MAX);
  pid_init(&angle_pid4, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_I_MAX, ANGLE_OUT_MAX);
  pid_init(&speed_pid4, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_I_MAX, SPEED_OUT_MAX);

  // BMI088 初始化（SPI1 就绪后调用）
  dbg_imu_error = BMI088_init();

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

  // Mahony 四元数初始值：单位四元数，表示无旋转
  // q[0]=w, q[1]=x, q[2]=y, q[3]=z
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick >= 10)   // 10ms 周期 = 100Hz，与 MahonyAHRS.c 的 sampleFreq 一致
    {
      last_tick = HAL_GetTick();

      const float dt = 0.01f;

      // ---- BMI088 读取 ----
      fp32 gyro[3], accel[3], temp;
      BMI088_read(gyro, accel, &temp);

      dbg_gyro_x  = gyro[0];
      dbg_gyro_y  = gyro[1];
      dbg_gyro_z  = gyro[2];
      dbg_accel_x = accel[0];
      dbg_accel_y = accel[1];
      dbg_accel_z = accel[2];
      dbg_temp    = temp;

      // ---- 上电陀螺零偏校准（前 GYRO_CALIB_MS ms 静止采集均值） ----
      if (!imu_calib_done)
      {
        // 累加（使用 double 防止 200 次累加的精度损失）
        static double bias_sum[3] = {0.0, 0.0, 0.0};
        static int    calib_cnt   = 0;
        const  int    calib_total = GYRO_CALIB_MS / 10;  // 10ms/次 → 200 次

        bias_sum[0] += gyro[0];
        bias_sum[1] += gyro[1];
        bias_sum[2] += gyro[2];
        calib_cnt++;
        dbg_calib_samples = calib_cnt;

        if (calib_cnt >= calib_total)
        {
          gyro_bias[0]   = (float)(bias_sum[0] / calib_cnt);
          gyro_bias[1]   = (float)(bias_sum[1] / calib_cnt);
          gyro_bias[2]   = (float)(bias_sum[2] / calib_cnt);

          // 校准结束时让 Mahony 先收敛一次，记录此刻姿态作为零点偏移
          // 用校准期间最后一帧加速度做一次更新，让四元数对齐重力方向
          float gx0 = gyro[1] - gyro_bias[1];
          float gy0 = gyro[0] - gyro_bias[0];
          float gz0 = gyro[2] - gyro_bias[2];
          float ax0 = accel[1], ay0 = accel[0], az0 = accel[2];
          // 多跑几次让四元数收敛到当前姿态
          for (int i = 0; i < 500; i++)
            MahonyAHRSupdateIMU(q, gx0, gy0, gz0, ax0, ay0, az0);

          // 记录此刻欧拉角为安装偏移
          float sin_p0 = 2.0f * (q[0]*q[2] - q[3]*q[1]);
          if      (sin_p0 >  1.0f) sin_p0 =  1.0f;
          else if (sin_p0 < -1.0f) sin_p0 = -1.0f;
          euler_offset[0] = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),
                                   1.0f-2.0f*(q[1]*q[1]+q[2]*q[2])) * rad_to_angle;
          euler_offset[1] = asinf(sin_p0) * rad_to_angle;
          euler_offset[2] = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]),
                                   1.0f-2.0f*(q[2]*q[2]+q[3]*q[3])) * rad_to_angle;

          imu_calib_done = 1;
        }
        // 校准期间不更新姿态，直接跳过后续 IMU 处理
      }
      else
      {
        // ---- 减去陀螺零偏 ----
        float gx = gyro[1] - gyro_bias[1];
        float gy = gyro[0] - gyro_bias[0];
        float gz = gyro[2] - gyro_bias[2];

        // ---- 加速度幅值门限：接近 1g 才用于姿态纠正 ----
        float accel_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
        float ax, ay, az;
        if (accel_norm >= ACCEL_VALID_MIN && accel_norm <= ACCEL_VALID_MAX)
        {
          // 幅值正常，使用真实加速度
          ax = accel[1];
          ay = accel[0];
          az = accel[2];
        }
        else
        {
          // 幅值异常（有线性加速度），传入全零让 Mahony 跳过加速度纠正
          ax = 0.0f; ay = 0.0f; az = 0.0f;
        }

        // ---- Mahony 姿态融合 ----
        MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);

        // ---- 四元数 → 欧拉角（ZYX，单位：度，范围 -180 ~ +180） ----
        {
          float sin_p = 2.0f * (q[0]*q[2] - q[3]*q[1]);
          if      (sin_p >  1.0f) sin_p =  1.0f;
          else if (sin_p < -1.0f) sin_p = -1.0f;

          float roll  = atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]),
                               1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])) * rad_to_angle;
          float pitch = asinf(sin_p) * rad_to_angle;
          float yaw   = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]),
                               1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * rad_to_angle;

          // 减去安装偏移，使上电姿态为零点
          roll  -= euler_offset[0];
          pitch -= euler_offset[1];
          yaw   -= euler_offset[2];

          // 规范化到 [-180, +180)
          if (roll  >  180.0f) roll  -= 360.0f;
          if (roll  < -180.0f) roll  += 360.0f;
          if (pitch >   90.0f) pitch  = 180.0f - pitch;
          if (pitch <  -90.0f) pitch  = -180.0f - pitch;
          if (yaw   >  180.0f) yaw   -= 360.0f;
          if (yaw   < -180.0f) yaw   += 360.0f;

          dbg_roll  = roll;
          dbg_pitch = pitch;
          dbg_yaw   = yaw;
        }
      }

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */