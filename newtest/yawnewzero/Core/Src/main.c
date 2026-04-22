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
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "AKF.h"   /* Gyro_AKF_HandleTypeDef / Gyro_AKF_Init / Gyro_AKF_Update */
#include "Config.h"     /* ParticleFilter_t / pf_* */

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

/* ===== 电机索引 ===================================================== */
#define MOTOR2_IDX  2
#define MOTOR4_IDX  4

/* ===== IMU 参数 ===================================================== */
/*
 *  GYRO_CALIB_MS      上电静止校准时长（ms）
 *                     校准期间同时用 Welford 算法在线计算三轴方差，
 *                     作为 AKF 的量测噪声方差 R0，无需手动填写。
 *  ACCEL_VALID_MIN/MAX 加速度幅值门限（m/s²）
 *                     超出范围时跳过加速度纠正，避免振动/冲击干扰。
 */
#define GYRO_CALIB_MS    2000    /* 2 秒 = 200 次采样（10ms/次）*/
#define ACCEL_VALID_MIN   5.0f  /* 约 0.82g */
#define ACCEL_VALID_MAX  15.0f  /* 约 1.12g */

/* ===== AKF 初始化参数 =============================================== */
/*
 *  AKF_P0   初始协方差（推荐 1.0）
 *  AKF_Q0   初始系统噪声方差（后续自适应，初值影响不大）
 *  R0 由校准阶段 Welford 在线计算，无需手动设置
 */
#define AKF_P0  1.0f
#define AKF_Q0  1e-7f

/* ===== 粒子滤波参数 ================================================= */
/*
 *  PF_NEFF_THRESH   有效粒子数阈值，低于此值触发重采样
 *                   推荐 PF_PARTICLE_NUM / 2
 */
#define PF_NEFF_THRESH  (PF_PARTICLE_NUM / 2.0f)

/* ===== 正弦扫描参数 ================================================= */
#define SCAN_AMP       40.0f
#define SCAN_FREQ       0.1f
#define SPEED_FF_GAIN   1.0f
#define HOMING_SPEED   20.0f
#define HOMING_SPD_LIMIT  50.0f   /* 归位阶段速度环目标限幅（rpm），调小可进一步减速 */
#define HOMING_THRESH   3.0f

/* ===== PID 参数 ===================================================== */
#define ANGLE_KP        10.0f
#define ANGLE_KI        0.05f
#define ANGLE_KD        0.4f
#define ANGLE_I_MAX    40.0f
#define ANGLE_OUT_MAX 250.0f

#define SPEED_KP      100.0f
#define SPEED_KI        0.5f
#define SPEED_KD        0.3f
#define SPEED_I_MAX  4000.0f
#define SPEED_OUT_MAX 2500.0f//25000

#define LPF_ALPHA  0.8f

/* 每个电机独立的 PID */
pid_struct_t angle_pid2, angle_pid4;
pid_struct_t speed_pid2, speed_pid4;

typedef enum {
  STATE_HOMING = 0,
  STATE_SCAN
} ctrl_state_t;

ctrl_state_t state2 = STATE_HOMING;
ctrl_state_t state4 = STATE_HOMING;

float prof_pos   = 0.0f;
float scan_phase = 0.0f;
float dbg_spd_ff = 0.0f;

float filtered_spd2 = 0.0f;
float filtered_spd4 = 0.0f;

uint16_t zero_enc2 = 0;
uint16_t zero_enc4 = 0;

/* ===== Ozone 观测变量 ===============================================
 *
 *  电机相关：
 *    dbg_angle2 / dbg_angle4      当前角度（°）
 *    dbg_prof                     共用虚拟目标
 *    dbg_err2 / dbg_err4          跟踪误差
 *    dbg_voltage2 / dbg_voltage4  输出电压
 *    dbg_state                    0=归位  1=扫描
 *
 *  BMI088 原始读出（BMI088_read 直接输出，未做任何处理）：
 *    dbg_gyro_x/y/z               rad/s
 *    dbg_accel_x/y/z              m/s²
 *    dbg_temp                     °C
 *    dbg_imu_error                0=初始化正常
 *
 *  AKF 滤波后陀螺仪（与原始值对比用）：
 *    dbg_gx_akf / dbg_gy_akf / dbg_gz_akf    rad/s
 *
 *  Mahony 输出（PF 之前，用于对比）：
 *    dbg_roll_mahony / dbg_pitch_mahony        °（相对重力方向的真实角度）
 *    dbg_yaw                                   °（不经 PF，直接输出）
 *
 *  PF 最终输出（接入控制用此值）：
 *    dbg_pitch / dbg_roll                      °（相对重力方向的真实角度）
 *
 *  校准信息：
 *    dbg_calib_samples            已采集样本数
 *    dbg_r0_gx / dbg_r0_gy / dbg_r0_gz   AKF 三轴 R0（方差）
 *
 * ================================================================== */
volatile float   dbg_angle2       = 0.0f;
volatile float   dbg_angle4       = 0.0f;
volatile float   dbg_prof         = 0.0f;
volatile float   dbg_err2         = 0.0f;
volatile float   dbg_err4         = 0.0f;
volatile float   dbg_voltage2     = 0.0f;
volatile float   dbg_voltage4     = 0.0f;
volatile int     dbg_state        = 0;

volatile fp32    dbg_gyro_x       = 0.0f;
volatile fp32    dbg_gyro_y       = 0.0f;
volatile fp32    dbg_gyro_z       = 0.0f;
volatile fp32    dbg_accel_x      = 0.0f;
volatile fp32    dbg_accel_y      = 0.0f;
volatile fp32    dbg_accel_z      = 0.0f;
volatile fp32    dbg_temp         = 0.0f;
volatile uint8_t dbg_imu_error    = 0;

volatile float   dbg_gx_akf       = 0.0f;
volatile float   dbg_gy_akf       = 0.0f;
volatile float   dbg_gz_akf       = 0.0f;

volatile float   dbg_roll_mahony  = 0.0f;
volatile float   dbg_pitch_mahony = 0.0f;
volatile float   dbg_roll         = 0.0f;
volatile float   dbg_pitch        = 0.0f;
volatile float   dbg_yaw          = 0.0f;

volatile int     dbg_calib_samples = 0;
volatile float   dbg_r0_gx        = 0.0f;
volatile float   dbg_r0_gy        = 0.0f;
volatile float   dbg_r0_gz        = 0.0f;

/* 陀螺仪零偏（校准均值） */
float gyro_bias[3]    = {0.0f, 0.0f, 0.0f};

volatile int imu_calib_done = 0;

/* ===== AKF 实例（三轴各一个）======================================= */
Gyro_AKF_HandleTypeDef akf_gx, akf_gy, akf_gz;

/* ===== 粒子滤波实例 ================================================= */
ParticleFilter_t pf;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN FPP */

/* USER CODE END FPP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 编码器原始值 → 角度（以各自零点为 0°） */
float enc_to_angle(uint16_t enc, uint16_t zero)
{
  int32_t diff = (int32_t)enc - (int32_t)zero;
  if      (diff >  4096) diff -= 8192;
  else if (diff < -4096) diff += 8192;
  return (float)diff * 360.0f / 8192.0f;
}

/* 角度误差计算（处理 ±180° 翻转） */
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

  /* MCU Configuration -------------------------------------------------------*/

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

  /* BMI088 初始化 */
  dbg_imu_error = BMI088_init();

  /* 等第一帧 CAN 数据到达 */
  HAL_Delay(100);

  /* 记录电机上电零点 */
  zero_enc2     = motor_info[MOTOR2_IDX].rotor_angle;
  zero_enc4     = motor_info[MOTOR4_IDX].rotor_angle;
  prof_pos      = 0.0f;
  filtered_spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
  filtered_spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Mahony 四元数初始值：单位四元数，表示无旋转（w,x,y,z） */
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  /* PF 首帧初始化标志（等 Mahony 收敛后再初始化） */
  uint8_t pf_initialized = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick >= 10)   /* 10ms = 100Hz，与 MahonyAHRS.c sampleFreq 一致 */
    {
      last_tick = HAL_GetTick();

      const float dt = 0.01f;

      /* ---------------------------------------------------------------- */
      /* 1. BMI088 读取                                                    */
      /* ---------------------------------------------------------------- */
      fp32 gyro[3], accel[3], temp;
      BMI088_read(gyro, accel, &temp);

      /* 原始数据挂 Ozone（未做任何处理） */
      dbg_gyro_x  = gyro[0];
      dbg_gyro_y  = gyro[1];
      dbg_gyro_z  = gyro[2];
      dbg_accel_x = accel[0];
      dbg_accel_y = accel[1];
      dbg_accel_z = accel[2];
      dbg_temp    = temp;

      /* ---------------------------------------------------------------- */
      /* 2. 上电校准（前 GYRO_CALIB_MS ms 静止）                          */
      /*    a) Welford 在线方差 → 三轴 R0（AKF 量测噪声）                 */
      /*    b) 零偏均值（200 次 double 累加）                              */
      /*    c) 校准结束：初始化三个 AKF，用加速度计均值初始化 Mahony 四元数*/
      /*       使 Mahony 从真实姿态出发积分，后续输出即为真实绝对角度      */
      /* ---------------------------------------------------------------- */
      if (!imu_calib_done)
      {
        /* 静态局部变量：整个校准期间持久化 */
        static double bias_sum[3]  = {0.0, 0.0, 0.0};  /* 陀螺零偏累加 */
        static double wf_mean[3]   = {0.0, 0.0, 0.0};  /* Welford 均值 */
        static double wf_M2[3]     = {0.0, 0.0, 0.0};  /* Welford 离均差平方和 */
        static double accel_sum[3] = {0.0, 0.0, 0.0};  /* 加速度计累加（用于初始化四元数）*/
        static int    calib_cnt    = 0;
        const  int    calib_total  = GYRO_CALIB_MS / 10; /* 200 次 */

        /* ---- a) 陀螺零偏累加 ---- */
        bias_sum[0] += (double)gyro[0];
        bias_sum[1] += (double)gyro[1];
        bias_sum[2] += (double)gyro[2];

        /* ---- b) 加速度计累加（用于校准结束时初始化 Mahony 四元数）---- */
        accel_sum[0] += (double)accel[0];
        accel_sum[1] += (double)accel[1];
        accel_sum[2] += (double)accel[2];

        /* ---- c) Welford 在线方差更新（三轴独立）→ AKF 的 R0 ---- */
        calib_cnt++;
        for (int axis = 0; axis < 3; axis++)
        {
          double val   = (double)gyro[axis];
          double delta = val - wf_mean[axis];
          wf_mean[axis] += delta / calib_cnt;
          double delta2  = val - wf_mean[axis];
          wf_M2[axis]   += delta * delta2;
        }

        dbg_calib_samples = calib_cnt;

        if (calib_cnt >= calib_total)
        {
          /* ---- 陀螺零偏均值 ---- */
          gyro_bias[0] = (float)(bias_sum[0] / calib_cnt);
          gyro_bias[1] = (float)(bias_sum[1] / calib_cnt);
          gyro_bias[2] = (float)(bias_sum[2] / calib_cnt);

          /* ---- 三轴 R0（无偏方差）---- */
          float r0_gx = (calib_cnt > 1) ? (float)(wf_M2[0] / (calib_cnt - 1)) : 1e-8f;
          float r0_gy = (calib_cnt > 1) ? (float)(wf_M2[1] / (calib_cnt - 1)) : 1e-8f;
          float r0_gz = (calib_cnt > 1) ? (float)(wf_M2[2] / (calib_cnt - 1)) : 1e-8f;
          /* 防止 R0 为 0，否则 AKF 增益变 1，退化为纯跟随 */
          if (r0_gx < 1e-10f) r0_gx = 1e-10f;
          if (r0_gy < 1e-10f) r0_gy = 1e-10f;
          if (r0_gz < 1e-10f) r0_gz = 1e-10f;

          dbg_r0_gx = r0_gx;
          dbg_r0_gy = r0_gy;
          dbg_r0_gz = r0_gz;

          /* ---- 初始化三个 AKF，x0 填零偏均值 ---- */
          Gyro_AKF_Init(&akf_gx, gyro_bias[0], AKF_P0, AKF_Q0, r0_gx);
          Gyro_AKF_Init(&akf_gy, gyro_bias[1], AKF_P0, AKF_Q0, r0_gy);
          Gyro_AKF_Init(&akf_gz, gyro_bias[2], AKF_P0, AKF_Q0, r0_gz);

          /* ---- 用加速度计 200 次均值初始化 Mahony 四元数 ---- */
          /*
           *  原理：校准期间设备静止，加速度计均值即为重力向量在机体系的投影。
           *  将此重力向量对应的姿态"写入"初始四元数，Mahony 从真实姿态出发
           *  积分，后续输出的欧拉角即为相对重力方向的绝对真实角度，
           *  而非相对上电时刻的相对角度。
           */
          float ax_mean = (float)(accel_sum[0] / calib_cnt);
          float ay_mean = (float)(accel_sum[1] / calib_cnt);
          float az_mean = (float)(accel_sum[2] / calib_cnt);

          /* 归一化到单位向量 */
          float a_norm = sqrtf(ax_mean*ax_mean + ay_mean*ay_mean + az_mean*az_mean);
          if (a_norm > 0.1f)
          {
            ax_mean /= a_norm;
            ay_mean /= a_norm;
            az_mean /= a_norm;
          }

          /* 由重力方向直接计算 roll/pitch 安装偏移
           * 轴映射：与正常运行段一致（pitch/roll 已对调），此处索引对调保持一致 */
          float roll_offset  = atan2f(ay_mean, az_mean);          /* 对调后 → roll  */
          float sin_p0       = -ax_mean;
          if      (sin_p0 >  1.0f) sin_p0 =  1.0f;
          else if (sin_p0 < -1.0f) sin_p0 = -1.0f;
          float pitch_offset = asinf(sin_p0);                     /* 对调后 → pitch */

          /* 用安装偏移构造初始四元数（ZYX，Yaw=0）
           * Mahony 从此姿态出发积分，输出即为真实绝对角度，
           * 运行段不再需要减任何 euler_offset */
          {
            float cr = cosf(roll_offset  * 0.5f);
            float sr = sinf(roll_offset  * 0.5f);
            float cp = cosf(pitch_offset * 0.5f);
            float sp = sinf(pitch_offset * 0.5f);
            q[0] =  cr * cp;
            q[1] =  sr * cp;
            q[2] =  cr * sp;
            q[3] = -sr * sp;
          }

          imu_calib_done = 1;
          /* IMU 校准结束时重新记录电机零点，使电机零点与 IMU 姿态零点对齐 */
          zero_enc2 = motor_info[MOTOR2_IDX].rotor_angle;
          zero_enc4 = motor_info[MOTOR4_IDX].rotor_angle;
          prof_pos  = 0.0f;
          /* pf_initialized 仍为 0，下一帧正常运行时再初始化 PF */
        }

        /* 校准期间跳过 IMU 更新和电机控制 */
        continue;
      }

      /* ---------------------------------------------------------------- */
      /* 3. 正常运行：AKF → 零偏补偿 → Mahony → 欧拉角 → PF            */
      /* ---------------------------------------------------------------- */

      /* ---- 3a. Gyro_AKF 滤波（原始域，减零偏之前）
       *          先 AKF 滤除随机噪声，再减静态零偏
       *          gz 不经 AKF 直接透传，避免动态旋转时幅值被压缩         */
      float gx_akf = Gyro_AKF_Update(&akf_gx, gyro[0]);
      float gy_akf = Gyro_AKF_Update(&akf_gy, gyro[1]);
      float gz_akf = gyro[2];   /* gz 不经 AKF，直接透传 */

      dbg_gx_akf = gx_akf;
      dbg_gy_akf = gy_akf;
      dbg_gz_akf = gz_akf;

      /* ---- 3b. 减零偏（不做轴交换，与加速度计保持一致）---- */
      float gx = gx_akf - gyro_bias[0];   /* Mahony gx ← BMI088 gyroX（AKF后） */
      float gy = gy_akf - gyro_bias[1];   /* Mahony gy ← BMI088 gyroY（AKF后） */
      float gz = gz_akf - gyro_bias[2];   /* Mahony gz ← BMI088 gyroZ */

      /* ---- 3c. 加速度幅值门限：接近 1g 才参与姿态纠正 ---- */
      float accel_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
      float ax, ay, az;
      if (accel_norm >= ACCEL_VALID_MIN && accel_norm <= ACCEL_VALID_MAX)
      {
        ax = accel[0];   /* Mahony ax ← BMI088 accelX */
        ay = accel[1];   /* Mahony ay ← BMI088 accelY */
        az = accel[2];
      }
      else
      {
        /* 幅值异常（有线性加速度/冲击），传入全零让 Mahony 跳过加速度纠正 */
        ax = 0.0f; ay = 0.0f; az = 0.0f;
      }

      /* ---- 3b_yaw. Yaw 轴 ZUPT：静止时估计 gz 残余零偏并实时补偿 -------
       *
       *  无磁力计时 yaw 靠陀螺 Z 轴积分，校准后仍有残余零偏导致 yaw 漂移。
       *  静止时 gz 理论为 0，实测值即残余零偏，用 PI 估计器慢慢收敛并减掉，
       *  直接作用在送入 Mahony 的 gz 上，四元数本身不再积累漂移。
       *
       *  参数：
       *    YAW_STATIC_THR   静止判断阈值（rad/s），推荐 0.03（≈1.7°/s）
       *    YAW_ZUPT_KP      比例增益，推荐 0.02
       *    YAW_ZUPT_KI      积分增益，推荐 0.001
       *    YAW_BIAS_MAX     零偏估计限幅（rad/s），推荐 0.05
       * ----------------------------------------------------------------- */
      #define YAW_STATIC_THR  0.1f
      #define YAW_ZUPT_KP     0.5f
      #define YAW_ZUPT_KI     0.05f
      #define YAW_BIAS_MAX    0.1f

      {
        /* 静止判断：AKF 后减零偏的三轴合矢量 */
        float gyro_sq = gx*gx + gy*gy + gz*gz;
        uint8_t yaw_static = (gyro_sq < (YAW_STATIC_THR * YAW_STATIC_THR));

        static float yaw_drift_bias = 0.0f;  /* gz 残余零偏估计（rad/s）*/
        static float yaw_drift_intg = 0.0f;  /* 积分项 */

        if (yaw_static)
        {
          /* 静止时 gz 残差即为零偏估计误差 */
          float gz_err       = gz - yaw_drift_bias;
          yaw_drift_intg    += YAW_ZUPT_KI * gz_err * dt;
          if      (yaw_drift_intg >  YAW_BIAS_MAX) yaw_drift_intg =  YAW_BIAS_MAX;
          else if (yaw_drift_intg < -YAW_BIAS_MAX) yaw_drift_intg = -YAW_BIAS_MAX;
          yaw_drift_bias     = YAW_ZUPT_KP * gz + yaw_drift_intg;
          if      (yaw_drift_bias >  YAW_BIAS_MAX) yaw_drift_bias =  YAW_BIAS_MAX;
          else if (yaw_drift_bias < -YAW_BIAS_MAX) yaw_drift_bias = -YAW_BIAS_MAX;
        }

        /* 从 gz 中减去估计的残余零偏，送入 Mahony 的陀螺不再漂移 */
        gz -= yaw_drift_bias;
      }

      /* ---- 3d. Mahony 姿态融合 ---- */
      MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);

      /* ---- 3e. 四元数 → 欧拉角（ZYX，°，-180~+180）----
       *
       *  初始四元数已在校准阶段由加速度计均值初始化为真实姿态，
       *  Mahony 从真实姿态积分，此处直接输出即为相对重力方向的绝对真实角度。
       *  不再需要减 euler_offset（已无此变量），无重复补偿问题。
       * ----------------------------------------------------------------- */
      float roll_m, pitch_m, yaw_m;
      {
        float sin_p = 2.0f * (q[0]*q[2] - q[3]*q[1]);
        if      (sin_p >  1.0f) sin_p =  1.0f;
        else if (sin_p < -1.0f) sin_p = -1.0f;

        roll_m  = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),
                         1.0f-2.0f*(q[1]*q[1]+q[2]*q[2])) * rad_to_angle;
        pitch_m = asinf(sin_p) * rad_to_angle;
        yaw_m   = -atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]),
                          1.0f-2.0f*(q[2]*q[2]+q[3]*q[3])) * rad_to_angle;

        /* pitch/roll 物理轴互换（安装方向决定）*/
        float tmp = roll_m;
        roll_m  = pitch_m;
        pitch_m = tmp;

        /* 归一化到 [-180, +180) */
        if      (roll_m  >  180.0f) roll_m  -= 360.0f;
        else if (roll_m  < -180.0f) roll_m  += 360.0f;
        if      (pitch_m >   90.0f) pitch_m  =  90.0f;   /* clamp，不做折叠 */
        else if (pitch_m <  -90.0f) pitch_m  = -90.0f;
        if      (yaw_m   >  180.0f) yaw_m   -= 360.0f;
        else if (yaw_m   < -180.0f) yaw_m   += 360.0f;
      }

      /* Mahony 直接输出挂 Ozone（PF 之前，用于对比）*/
      dbg_roll_mahony  = roll_m;
      dbg_pitch_mahony = pitch_m;
      dbg_yaw          = yaw_m;   /* Yaw 不经 PF，直接输出 */

      /* ---- 3f. 粒子滤波（Pitch / Roll）---- */
      if (!pf_initialized)
      {
        /* 首次进入正常运行：用 Mahony 当前输出初始化 PF */
        pf_init(&pf, pitch_m, roll_m);
        pf_initialized = 1;
      }
      else
      {
        /* 预测：粒子加过程噪声 */
        pf_predict(&pf);

        /* 更新：用 Mahony 欧拉角作为观测量 */
        pf_update(&pf, pitch_m, roll_m);

        /* 有效粒子数检测，退化则重采样 */
        float neff = pf_calculate_neff(&pf);
        if (neff < PF_NEFF_THRESH)
        {
          pf_resample(&pf);
        }

        /* 加权均值估计 → pf.state_est_deg[] */
        pf_estimate(&pf);
      }

      /* PF 最终输出挂 Ozone（暂时旁路 PF，直接用 Mahony 输出）*/
      dbg_pitch = pitch_m;
      dbg_roll  = roll_m;

      /* ---------------------------------------------------------------- */
      /* 4. 电机控制                                                       */
      /* ---------------------------------------------------------------- */

#if OPEN_LOOP_TEST
      set_motor_voltage(0, 0, (int16_t)OPEN_LOOP_VOLTAGE,
                              0,
                              (int16_t)OPEN_LOOP_VOLTAGE);
#else
      /* 读取两个电机反馈 */
      dbg_angle2 = enc_to_angle(motor_info[MOTOR2_IDX].rotor_angle, zero_enc2);
      dbg_angle4 = enc_to_angle(motor_info[MOTOR4_IDX].rotor_angle, zero_enc4);

      float spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
      float spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;

      filtered_spd2 = LPF_ALPHA * spd2 + (1.0f - LPF_ALPHA) * filtered_spd2;
      filtered_spd4 = LPF_ALPHA * spd4 + (1.0f - LPF_ALPHA) * filtered_spd4;

      /* ----------------------------------------------------------------
       * 归位阶段
       *   目标：让 IMU 对应轴角度收敛到 0°
       *   ID2 ← Yaw（dbg_yaw），ID4 ← Pitch（dbg_pitch）
       *   外环误差直接来自 IMU 角度，内环仍为电机转速
       *
       *   两路必须同时到位（both_scanning == 1）才切入哨兵扫描模式，
       *   先到位的那路继续保持 0° 等待另一路，不提前跟 prof_pos。
       * -------------------------------------------------------------- */

      /* ID2 归位完成判断：Yaw 进入阈值 */
      if (state2 == STATE_HOMING)
      {
        if (fabsf(dbg_yaw) < HOMING_THRESH)
          state2 = STATE_SCAN;
      }

      /* ID4 归位完成判断：Pitch 进入阈值 */
      if (state4 == STATE_HOMING)
      {
        if (fabsf(dbg_pitch) < HOMING_THRESH)
          state4 = STATE_SCAN;
      }

      /* 两路都就绪才算真正进入扫描 */
      int both_scanning = (state2 == STATE_SCAN && state4 == STATE_SCAN);
      dbg_state = both_scanning;

      /* 检测 both_scanning 上升沿：归位刚结束时清零所有 PID 积分项，
       * 防止归位阶段积累的 i_out 在切换瞬间造成输出突变（抽搐）        */
      static int prev_both_scanning = 0;
      if (both_scanning && !prev_both_scanning)
      {
        angle_pid2.i_out = 0.0f;
        speed_pid2.i_out = 0.0f;
        angle_pid4.i_out = 0.0f;
        speed_pid4.i_out = 0.0f;
        scan_phase       = 0.0f;   /* 正弦从 0 重新开始，确保 prof_pos 平滑起步 */
      }
      prev_both_scanning = both_scanning;

      if (both_scanning)
      {
        scan_phase += dt;
        float omega    = 2.0f * (float)M_PI * SCAN_FREQ;
        prof_pos       = SCAN_AMP * sinf(omega * scan_phase);
        float ff_deg_s = SCAN_AMP * omega * cosf(omega * scan_phase);
        dbg_spd_ff     = ff_deg_s * SPEED_FF_GAIN;
      }
      else
      {
        /* 任一路仍在归位，prof_pos 锁 0，前馈清零，scan_phase 不推进 */
        prof_pos   = 0.0f;
        dbg_spd_ff = 0.0f;
      }

      dbg_prof = prof_pos;

      /* ---- ID2 PID（外环误差 = IMU Yaw）----
       *   归位 / 等待期间：目标 0°，限速 HOMING_SPD_LIMIT
       *   both_scanning 后：目标 prof_pos，限速恢复正常              */
      {
        float imu_target2 = both_scanning ? prof_pos : 0.0f;
        dbg_err2          = angle_err_calc(imu_target2, dbg_yaw);
        angle_pid2.err[1] = angle_pid2.err[0];
        angle_pid2.err[0] = dbg_err2;
        angle_pid2.p_out  = angle_pid2.kp * angle_pid2.err[0];
        angle_pid2.i_out += angle_pid2.ki * angle_pid2.err[0];
        LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
        angle_pid2.d_out  = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
        float pid_spd2    = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;

        float spd_ff2     = both_scanning ? dbg_spd_ff : 0.0f;
        float spd_target2 = pid_spd2 + spd_ff2;
        /* 归位 / 等待期间限速，both_scanning 后恢复正常限幅 */
        float spd_lim2    = both_scanning ? angle_pid2.out_max : HOMING_SPD_LIMIT;
        LIMIT_MIN_MAX(spd_target2, -spd_lim2, spd_lim2);

        float spd_err2    = spd_target2 - filtered_spd2;
        speed_pid2.err[1] = speed_pid2.err[0];
        speed_pid2.err[0] = spd_err2;
        speed_pid2.p_out  = speed_pid2.kp * speed_pid2.err[0];
        speed_pid2.i_out += speed_pid2.ki * speed_pid2.err[0];
        LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);
        speed_pid2.d_out  = speed_pid2.kd * (speed_pid2.err[0] - speed_pid2.err[1]);
        dbg_voltage2      = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;
        LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);
      }

      /* ---- ID4 PID（外环误差 = IMU Pitch）----
       *   归位 / 等待期间：目标 0°，限速 HOMING_SPD_LIMIT
       *   both_scanning 后：目标 prof_pos，限速恢复正常              */
      {
        float imu_target4 = both_scanning ? prof_pos : 0.0f;
        dbg_err4          = angle_err_calc(imu_target4, dbg_pitch);
        angle_pid4.err[1] = angle_pid4.err[0];
        angle_pid4.err[0] = dbg_err4;
        angle_pid4.p_out  = angle_pid4.kp * angle_pid4.err[0];
        angle_pid4.i_out += angle_pid4.ki * angle_pid4.err[0];
        LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);
        angle_pid4.d_out  = angle_pid4.kd * (angle_pid4.err[0] - angle_pid4.err[1]);
        float pid_spd4    = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;

        float spd_ff4     = both_scanning ? dbg_spd_ff : 0.0f;
        float spd_target4 = pid_spd4 + spd_ff4;
        /* 归位 / 等待期间限速，both_scanning 后恢复正常限幅 */
        float spd_lim4    = both_scanning ? angle_pid4.out_max : HOMING_SPD_LIMIT;
        LIMIT_MIN_MAX(spd_target4, -spd_lim4, spd_lim4);

        float spd_err4    = spd_target4 - filtered_spd4;
        speed_pid4.err[1] = speed_pid4.err[0];
        speed_pid4.err[0] = spd_err4;
        speed_pid4.p_out  = speed_pid4.kp * speed_pid4.err[0];
        speed_pid4.i_out += speed_pid4.ki * speed_pid4.err[0];
        LIMIT_MIN_MAX(speed_pid4.i_out, -speed_pid4.i_max, speed_pid4.i_max);
        speed_pid4.d_out  = speed_pid4.kd * (speed_pid4.err[0] - speed_pid4.err[1]);
        dbg_voltage4      = speed_pid4.p_out + speed_pid4.i_out + speed_pid4.d_out;
        LIMIT_MIN_MAX(dbg_voltage4, -speed_pid4.out_max, speed_pid4.out_max);
      }

      set_motor_voltage(0,
                        0,
                        (int16_t)dbg_voltage2,
                        0,
                        (int16_t)dbg_voltage4);
#endif

    } /* end if (tick >= 10) */

  } /* end while(1) */
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

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM           = 8;
  RCC_OscInitStruct.PLL.PLLN           = 168;
  RCC_OscInitStruct.PLL.PLLP           = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ           = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
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