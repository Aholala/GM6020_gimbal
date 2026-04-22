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
#include "usb_device.h"
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
#define MOTOR2_IDX  2   /* Yaw 轴 */
#define MOTOR4_IDX  4   /* Pitch 轴 */

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
#define AKF_Q0  1e-5f

/* ===== 粒子滤波参数 ================================================= */
/*
 *  PF_NEFF_THRESH   有效粒子数阈值，低于此值触发重采样
 *                   推荐 PF_PARTICLE_NUM / 2
 */
#define PF_NEFF_THRESH  (PF_PARTICLE_NUM / 2.0f)

/* ===== 正弦扫描参数 ================================================= */
#define SCAN_AMP       30.0f
#define SCAN_FREQ       0.1f
#define SPEED_FF_GAIN   1.0f
#define HOMING_SPEED   20.0f
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
#define SPEED_OUT_MAX 10000.0f//25000

#define LPF_ALPHA  0.8f

/* 每个电机独立的 PID */
pid_struct_t angle_pid2, angle_pid4;
pid_struct_t speed_pid2, speed_pid4;

/* ===== 顶层状态机 ===================================================
 *
 *  TOP_IMU_ALIGN  IMU 校准完成后，先把云台打到 IMU 零点（pitch=0, yaw=0）
 *                 motor2（yaw）目标 = -dbg_yaw
 *                 motor4（pitch）目标 = -dbg_pitch
 *                 两轴误差均小于 HOMING_THRESH 后进入 TOP_SCAN
 *
 *  TOP_SCAN       执行原有的 encoder 归位 + 正弦扫描逻辑
 *
 * ================================================================== */
typedef enum {
  TOP_IMU_ALIGN = 0,
  TOP_SCAN
} top_state_t;

top_state_t top_state = TOP_IMU_ALIGN;

/* ===== IMU 对齐阶段虚拟目标（梯形速度规划用）========================
 *
 *  prof_yaw   motor2（yaw 轴）的虚拟跟踪目标，以 HOMING_SPEED 步进逼近
 *             真实目标 -dbg_yaw，避免上电瞬间大误差冲击电机。
 *  prof_pitch motor4（pitch 轴）同理。
 *  两个变量在 IMU 校准完成时初始化为当前编码器角度（即 0），
 *  每帧以不超过 HOMING_SPEED * dt 的步长向目标靠近。
 * ================================================================== */
float prof_yaw   = 0.0f;
float prof_pitch = 0.0f;

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
 *    dbg_state                    0=IMU对齐  1=encoder归位  2=扫描
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
 *  安装偏置（校准结束时自动记录，Ozone 可观测）：
 *    dbg_pitch_offset / dbg_roll_offset        °
*   视觉下行帧（BSP_USB_Receive 解析结果，每帧刷新）：
 *    dbg_vis_detected             是否检测到目标（0/1）
 *    dbg_vis_yaw                  视觉下发 yaw（单位与视觉协议一致）
 *    dbg_vis_pitch                视觉下发 pitch（单位与视觉协议一致）
 *
 *  视觉接收诊断（定义在 bsp_usb.c，此处 extern 供 Ozone 直接观测）：
 *    vis_rx_total                 收到的总帧次（应持续递增）
 *    vis_rx_ok                    解析成功次数（正常应 == vis_rx_total）
 *    vis_rx_badlen                长度错误次数（正常应为 0，若非 0 说明帧长不是 11）
 *    vis_rx_badhdr                帧头错误次数（正常应为 0）
 *    vis_rx_badchk                校验错误次数（正常应为 0）
 *    vis_rx_lastlen               最近一次收到的原始长度（应为 11）
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

/* 安装偏置（校准结束时自动记录，Ozone 可观测验证） */
volatile float   dbg_pitch_offset = 0.0f;
volatile float   dbg_roll_offset  = 0.0f;

/* 视觉下行帧观测 */
volatile uint8_t dbg_vis_detected = 0;      /* 是否检测到目标（0/1）*/
volatile float   dbg_vis_yaw      = 0.0f;   /* 视觉下发 yaw（单位与视觉协议一致）*/
volatile float   dbg_vis_pitch    = 0.0f;   /* 视觉下发 pitch（单位与视觉协议一致）*/

/* 视觉接收诊断（定义在 bsp_usb.c，此处 extern 供 Ozone 观测）*/
extern volatile uint32_t vis_rx_total;    /* 收到的总帧次 */
extern volatile uint32_t vis_rx_ok;       /* 解析成功次数 */
extern volatile uint32_t vis_rx_badlen;   /* 长度错误次数 */
extern volatile uint32_t vis_rx_badhdr;   /* 帧头错误次数 */
extern volatile uint32_t vis_rx_badchk;   /* 校验错误次数 */
extern volatile uint32_t vis_rx_lastlen;  /* 最近一次收到的原始长度 */
extern volatile uint8_t  vis_rx_raw[16];
extern volatile uint32_t vis_rx_raw_len;
/* 陀螺仪零偏（校准均值） */
float gyro_bias[3]    = {0.0f, 0.0f, 0.0f};

volatile int imu_calib_done = 0;

/* ===== IMU 安装偏置（校准结束时由加速度计自动计算）================= */
/*
 *  上电时云台停在编码器零点，校准结束后记录此刻 IMU 输出的 pitch/roll，
 *  运行段每帧减掉，使 IMU 零点与编码器零点对齐。
 *  无需手动填写，每次上电自动更新。
 */
float imu_pitch_offset = 0.0f;
float imu_roll_offset  = 0.0f;

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

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
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
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
    if (HAL_GetTick() - last_tick >= 1)   /* 10ms = 100Hz，与 MahonyAHRS.c sampleFreq 一致 */
    {
      last_tick = HAL_GetTick();

      const float dt = 0.001f;

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
      /*    d) 校准结束：记录此刻欧拉角作为安装偏置，运行段减掉            */
      /*       前提：上电时云台停在编码器零点位置                          */
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

          /* ---- d) 记录安装偏置 ----------------------------------------
           *
           *  初始四元数已写入，此刻做一次欧拉角正解，得到 IMU 在当前安装
           *  姿态下的 pitch/roll 输出值，即为安装偏置。
           *  运行段每帧减掉，使 IMU 零点与编码器零点自动对齐。
           *  前提：上电时云台停在编码器零点。
           * ---------------------------------------------------------------- */
          {
            float sin_p = 2.0f * (q[0]*q[2] - q[3]*q[1]);
            if      (sin_p >  1.0f) sin_p =  1.0f;
            else if (sin_p < -1.0f) sin_p = -1.0f;

            float r0 = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),
                              1.0f-2.0f*(q[1]*q[1]+q[2]*q[2])) * rad_to_angle;
            float p0 = asinf(sin_p) * rad_to_angle;

            /* pitch/roll 轴互换，与运行段保持一致 */
            float tmp = r0; r0 = p0; p0 = tmp;

            imu_pitch_offset = p0;
            imu_roll_offset  = r0;

            /* 挂 Ozone，便于验证偏置是否合理 */
            dbg_pitch_offset = imu_pitch_offset;
            dbg_roll_offset  = imu_roll_offset;
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
      /* ---- 3b_yaw. 增强版 ZUPT：动态追踪残余零偏 ----------------------- */
      /* ---- 3b_yaw. 优化版 ZUPT：平衡稳定性与跟随性 ----------------------- */
      // 1. 收紧静止判断：只有在非常静止时才更新零偏，防止运动被误判为漂移
#define YAW_STATIC_THR  0.015f   // 约 0.8°/s，确保手部微动都不会触发
      // 2. 降低反馈强度：减小 KI，防止补偿值学得太快导致"反向拖拽"
#define YAW_ZUPT_KP     0.1f    /* 降低比例项，减少瞬态干扰 */
#define YAW_ZUPT_KI     0.001f   // 极小的积分增益，仅用于对抗长期的温漂

#define YAW_BIAS_MAX    0.1f    /* 限制最大补偿值 */

      {
        static float yaw_drift_bias = 0.0f;
        static float yaw_drift_intg = 0.0f;

        float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);
        uint8_t is_static = (gyro_norm < YAW_STATIC_THR);

        if (is_static) {
          // 只有静止时，缓慢学习零偏
          yaw_drift_intg += YAW_ZUPT_KI * gz * dt;
          // 限制补偿范围，防止异常值干扰
          if (yaw_drift_intg > 0.05f) yaw_drift_intg = 0.05f;
          if (yaw_drift_intg < -0.05f) yaw_drift_intg = -0.05f;

          yaw_drift_bias = yaw_drift_intg;
        }
        // 注意：else 分支不进行任何操作，锁定上一次的 yaw_drift_bias

        gz -= yaw_drift_bias;
        gz *= 0.88f;
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

      /* Mahony 直接输出挂 Ozone（安装偏置补偿之前，用于对比）*/
      dbg_roll_mahony  = roll_m;
      dbg_pitch_mahony = pitch_m;
      dbg_yaw          = yaw_m;   /* Yaw 不经 PF，直接输出 */

      /* ---- 3e_offset. 安装偏置补偿 ------------------------------------
       *
       *  上电校准结束时记录的 imu_pitch_offset / imu_roll_offset，
       *  即为云台在编码器零点时 IMU 的输出值。
       *  减掉后 IMU 零点与编码器零点自动对齐，无需手动填数。
       * ----------------------------------------------------------------- */
      pitch_m -= imu_pitch_offset;
      roll_m  -= imu_roll_offset;

      /* 补偿后重新归一化 */
      if      (pitch_m >  90.0f) pitch_m =  90.0f;
      else if (pitch_m < -90.0f) pitch_m = -90.0f;
      if      (roll_m  >  180.0f) roll_m -= 360.0f;
      else if (roll_m  < -180.0f) roll_m += 360.0f;

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

      /* ---------------------------------------------------------------- */
      /* 4a. TOP_IMU_ALIGN：把云台转到 IMU 零点（pitch=0, yaw=0）         */
      /*                                                                   */
      /*  真实目标：motor2（yaw）→ -dbg_yaw，motor4（pitch）→ -dbg_pitch  */
      /*  虚拟目标：prof_yaw / prof_pitch 以 HOMING_SPEED 梯形步进逼近，   */
      /*            避免上电大误差直接冲击电机。                           */
      /*  两轴 IMU 角度均小于 HOMING_THRESH 后切换至 TOP_SCAN。            */
      /* ---------------------------------------------------------------- */
      if (top_state == TOP_IMU_ALIGN)
      {
        dbg_state = 0;   /* Ozone：0 = IMU 对齐 */

        float step = HOMING_SPEED * dt;

        /* ---- 梯形规划：prof_yaw 逼近 -dbg_yaw ---- */
        {
          float real_target2 = -dbg_yaw;
          float diff2 = angle_err_calc(real_target2, prof_yaw);
          if      (diff2 >  step) prof_yaw += step;
          else if (diff2 < -step) prof_yaw -= step;
          else                    prof_yaw  = real_target2;
        }

        /* ---- 梯形规划：prof_pitch 逼近 -dbg_pitch ---- */
        {
          float real_target4 = -dbg_pitch;
          float diff4 = angle_err_calc(real_target4, prof_pitch);
          if      (diff4 >  step) prof_pitch += step;
          else if (diff4 < -step) prof_pitch -= step;
          else                    prof_pitch  = real_target4;
        }

        /* ---- ID2 PID（yaw 轴 IMU 对齐，跟踪 prof_yaw）---- */
        {
          dbg_err2          = angle_err_calc(prof_yaw, dbg_angle2);
          angle_pid2.err[1] = angle_pid2.err[0];
          angle_pid2.err[0] = dbg_err2;
          angle_pid2.p_out  = angle_pid2.kp * angle_pid2.err[0];
          angle_pid2.i_out += angle_pid2.ki * angle_pid2.err[0];
          LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
          angle_pid2.d_out  = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
          float pid_spd2    = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;
          LIMIT_MIN_MAX(pid_spd2, -angle_pid2.out_max, angle_pid2.out_max);

          float spd_err2    = pid_spd2 - filtered_spd2;
          speed_pid2.err[1] = speed_pid2.err[0];
          speed_pid2.err[0] = spd_err2;
          speed_pid2.p_out  = speed_pid2.kp * speed_pid2.err[0];
          speed_pid2.i_out += speed_pid2.ki * speed_pid2.err[0];
          LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);
          speed_pid2.d_out  = speed_pid2.kd * (speed_pid2.err[0] - speed_pid2.err[1]);
          dbg_voltage2      = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;
          LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);
        }

        /* ---- ID4 PID（pitch 轴 IMU 对齐，跟踪 prof_pitch）---- */
        {
          dbg_err4          = angle_err_calc(prof_pitch, dbg_angle4);
          angle_pid4.err[1] = angle_pid4.err[0];
          angle_pid4.err[0] = dbg_err4;
          angle_pid4.p_out  = angle_pid4.kp * angle_pid4.err[0];
          angle_pid4.i_out += angle_pid4.ki * angle_pid4.err[0];
          LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);
          angle_pid4.d_out  = angle_pid4.kd * (angle_pid4.err[0] - angle_pid4.err[1]);
          float pid_spd4    = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;
          LIMIT_MIN_MAX(pid_spd4, -angle_pid4.out_max, angle_pid4.out_max);

          float spd_err4    = pid_spd4 - filtered_spd4;
          speed_pid4.err[1] = speed_pid4.err[0];
          speed_pid4.err[0] = spd_err4;
          speed_pid4.p_out  = speed_pid4.kp * speed_pid4.err[0];
          speed_pid4.i_out += speed_pid4.ki * speed_pid4.err[0];
          LIMIT_MIN_MAX(speed_pid4.i_out, -speed_pid4.i_max, speed_pid4.i_max);
          speed_pid4.d_out  = speed_pid4.kd * (speed_pid4.err[0] - speed_pid4.err[1]);
          dbg_voltage4      = speed_pid4.p_out + speed_pid4.i_out + speed_pid4.d_out;
          LIMIT_MIN_MAX(dbg_voltage4, -speed_pid4.out_max, speed_pid4.out_max);
        }

        /* 两轴 IMU 角度均到位，切换到扫描阶段 */
        if (fabsf(dbg_pitch) < HOMING_THRESH && fabsf(dbg_yaw) < HOMING_THRESH)
        {
          /* 以当前编码器位置为新的扫描零点，后续 prof_pos 从 0 出发 */
          zero_enc2  = motor_info[MOTOR2_IDX].rotor_angle;
          zero_enc4  = motor_info[MOTOR4_IDX].rotor_angle;
          prof_pos   = 0.0f;
          scan_phase = 0.0f;
          dbg_spd_ff = 0.0f;
          state2     = STATE_HOMING;
          state4     = STATE_HOMING;
          /* PID 积分清零，避免切换瞬间有积分残留冲击 */
          angle_pid2.i_out = 0.0f; speed_pid2.i_out = 0.0f;
          angle_pid4.i_out = 0.0f; speed_pid4.i_out = 0.0f;
          top_state  = TOP_SCAN;
        }
      }
      /* ---------------------------------------------------------------- */
      /* 4b. TOP_SCAN：encoder 归位 + 正弦扫描（原有逻辑）               */
      /* ---------------------------------------------------------------- */
      else /* top_state == TOP_SCAN */
      {
        /* 归位阶段 */
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

        int both_scanning = (state2 == STATE_SCAN && state4 == STATE_SCAN);
        dbg_state = both_scanning ? 2 : 1;   /* Ozone：1=encoder归位  2=扫描 */

        if (both_scanning)
        {
          scan_phase += dt;
          float omega    = 2.0f * (float)M_PI * SCAN_FREQ;
          prof_pos       = SCAN_AMP * sinf(omega * scan_phase);
          float ff_deg_s = SCAN_AMP * omega * cosf(omega * scan_phase);
          dbg_spd_ff     = ff_deg_s * SPEED_FF_GAIN;
        }

        dbg_prof = prof_pos;

        /* ---- ID2 PID ---- */
        {
          dbg_err2          = angle_err_calc(-prof_pos, dbg_angle2);
          angle_pid2.err[1] = angle_pid2.err[0];
          angle_pid2.err[0] = dbg_err2;
          angle_pid2.p_out  = angle_pid2.kp * angle_pid2.err[0];
          angle_pid2.i_out += angle_pid2.ki * angle_pid2.err[0];
          LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
          angle_pid2.d_out  = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
          float pid_spd2    = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;

          float spd_target2 = pid_spd2 + dbg_spd_ff;
          LIMIT_MIN_MAX(spd_target2, -angle_pid2.out_max, angle_pid2.out_max);

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

        /* ---- ID4 PID ---- */
        {
          dbg_err4          = angle_err_calc(-prof_pos, dbg_angle4);
          angle_pid4.err[1] = angle_pid4.err[0];
          angle_pid4.err[0] = dbg_err4;
          angle_pid4.p_out  = angle_pid4.kp * angle_pid4.err[0];
          angle_pid4.i_out += angle_pid4.ki * angle_pid4.err[0];
          LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);
          angle_pid4.d_out  = angle_pid4.kd * (angle_pid4.err[0] - angle_pid4.err[1]);
          float pid_spd4    = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;

          float spd_target4 = pid_spd4 + dbg_spd_ff;
          LIMIT_MIN_MAX(spd_target4, -angle_pid4.out_max, angle_pid4.out_max);

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
      } /* end TOP_SCAN */

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
