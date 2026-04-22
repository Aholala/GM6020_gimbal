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
#include "bsp_usb.h"
#include "bsp_bmi088driver.h"
#include "module_pid.h"
#include "lib_MahonyAHRS.h"
#include "lib_akf.h"
#include "lib_ahrsfusion.h"



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
/* 1=开环测试（固定电压），0=闭环控制 */
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
#define SPEED_FF_GAIN   0.0f
#define HOMING_SPEED   20.0f
#define HOMING_THRESH   3.0f

/* ===== PID 参数 ===================================================== */
#define ANGLE_KP        6.0f
#define ANGLE_KI        0.5f
#define ANGLE_KD        0.0f
#define ANGLE_I_MAX    40.0f
#define ANGLE_OUT_MAX 250.0f

#define SPEED_KP      100.0f
#define SPEED_KI        0.05f
#define SPEED_KD        0.0f
#define SPEED_I_MAX  4000.0f
#define SPEED_OUT_MAX 10000.0f//25000

#define LPF_ALPHA  0.5f

/* ===== 自瞄 / 哨兵模式切换参数 ===================================== */
/*
 *  TRACK_LIMIT_YAW    ID2（yaw 轴）自瞄限位（°）
 *  TRACK_LIMIT_PITCH  ID4（pitch 轴）自瞄限位（°）
 *  LOST_TIMEOUT_MS    丢目标超时时间（ms），超过后从自瞄切回归零
 *  RETURN_THRESH      回零判定阈值（°），两轴均满足才切扫描
 */
#define TRACK_LIMIT_YAW    60.0f
#define TRACK_LIMIT_PITCH  40.0f
#define LOST_TIMEOUT_MS    500
#define RETURN_THRESH       3.0f

/* 每个电机独立的 PID */
pid_struct_t angle_pid2, angle_pid4;
pid_struct_t speed_pid2, speed_pid4;

/* ===== 主状态机 ====================================================
 *
 *  STATE_HOMING    上电归零（prof_pos 缓慢移向 0°）
 *  STATE_SCAN      哨兵扫描（正弦轨迹，未识别到装甲板）
 *  STATE_RETURNING 丢目标后回零（目标固定为 0°，等待到位后切扫描）
 *  STATE_TRACK     自瞄跟随（视觉角度直接作为目标位置，带限位保护）
 *
 * ================================================================= */
typedef enum {
  STATE_HOMING = 0,
  STATE_SCAN,
  STATE_RETURNING,
  STATE_TRACK
} ctrl_state_t;

ctrl_state_t state2 = STATE_HOMING;
ctrl_state_t state4 = STATE_HOMING;

float prof_pos   = 0.0f;
float scan_phase = 0.0f;
float dbg_spd_ff = 0.0f;

/* 归零阶段各轴独立虚拟目标（°）
 *   STATE_HOMING / STATE_RETURNING 时使用，与 prof_pos 互不干扰
 *   STATE_SCAN / STATE_TRACK 时不再使用（由 prof_pos 或视觉角度接管）*/
float homing_target2 = 0.0f;   /* ID2 yaw  轴归零虚拟目标 */
float homing_target4 = 0.0f;   /* ID4 pitch 轴归零虚拟目标 */

float filtered_spd2 = 0.0f;
float filtered_spd4 = 0.0f;

uint16_t zero_enc2 = 0;
uint16_t zero_enc4 = 0;

/* 自瞄目标角度（°），由视觉弧度转换而来，经限位 clamp 后使用 */
float track_target_yaw   = 0.0f;  /* ID2 yaw 目标 */
float track_target_pitch = 0.0f;  /* ID4 pitch 目标 */

/* 丢目标计时器（ms） */
uint32_t lost_timer_ms = 0;

/* ===== Ozone 观测变量 ===============================================
 *
 *  电机相关：
 *    dbg_angle2 / dbg_angle4      当前角度（°）
 *    dbg_prof                     扫描阶段共用虚拟目标（STATE_SCAN）
 *                                 归零阶段请分别观测 homing_target2/4
 *    dbg_err2 / dbg_err4          跟踪误差
 *    dbg_voltage2 / dbg_voltage4  输出电压
 *    dbg_state                    0=归位  1=扫描  2=回零  3=自瞄
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
 *    dbg_roll_mahony / dbg_pitch_mahony        °
 *    dbg_yaw                                   °（不经 PF，直接输出）
 *
 *  PF 最终输出（接入控制用此值）：
 *    dbg_pitch / dbg_roll                      °
 *
 *  校准信息：
 *    dbg_calib_samples            已采集样本数
 *    dbg_r0_gx / dbg_r0_gy / dbg_r0_gz   AKF 三轴 R0（方差）
 *
 *  视觉下行帧（BSP_USB_Receive 解析结果，每帧刷新）：
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
volatile float   dbg_yaw          = 0.0f;   /* IMU Mahony yaw（仅供对比观测，不参与控制）*/
volatile float   dbg_yaw_enc      = 0.0f;   /* 编码器 yaw（ID2 实际反馈，控制用此值）*/

volatile int     dbg_calib_samples = 0;
volatile float   dbg_r0_gx        = 0.0f;
volatile float   dbg_r0_gy        = 0.0f;
volatile float   dbg_r0_gz        = 0.0f;

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
/* 安装偏移（校准收敛后记录，之后欧拉角减掉此值） */
float euler_offset[3] = {0.0f, 0.0f, 0.0f};  /* [roll, pitch, yaw] */

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
  /* 归零虚拟目标初始化为上电时的编码器角度（以零点为参考即 0°），
     上电后若电机已在零点附近则直接满足归零条件，不会产生初始突变 */
  homing_target2 = 0.0f;
  homing_target4 = 0.0f;
  filtered_spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
  filtered_spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;

  BSP_USB_Init();
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
    /* 接收并解析视觉下行帧 */
    BSP_USB_Receive();

    /* 视觉数据挂 Ozone（每帧刷新，不受 10ms 节拍限制）*/
    dbg_vis_detected = g_vision_rx.detected;
    dbg_vis_yaw      = g_vision_rx.yaw_rad;
    dbg_vis_pitch    = g_vision_rx.pitch_rad;

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
      /*    c) 校准结束：初始化三个 AKF，让 Mahony 收敛，记录安装偏移      */
      /* ---------------------------------------------------------------- */
      if (!imu_calib_done)
      {
        /* 静态局部变量：整个校准期间持久化 */
        static double bias_sum[3]  = {0.0, 0.0, 0.0};  /* 陀螺零偏累加 */
        static double wf_mean[3]   = {0.0, 0.0, 0.0};  /* Welford 均值 */
        static double wf_M2[3]     = {0.0, 0.0, 0.0};  /* Welford 离均差平方和 */
        static double accel_sum[3] = {0.0, 0.0, 0.0};  /* 加速度计累加（用于算安装偏移）*/
        static int    calib_cnt    = 0;
        const  int    calib_total  = GYRO_CALIB_MS / 10; /* 200 次 */

        /* ---- a) 陀螺零偏累加 ---- */
        bias_sum[0] += (double)gyro[0];
        bias_sum[1] += (double)gyro[1];
        bias_sum[2] += (double)gyro[2];

        /* ---- b) 加速度计累加（用于校准结束时算安装偏移）---- */
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

          /* ---- 用加速度计 200 次均值直接计算安装偏移 ----
           *
           *  轴映射与正常运行段保持一致（X/Y 交换）。
           * -------------------------------------------------------- */
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

          /* 轴映射：与正常运行段一致，不做交换 */
          float ax0 = ax_mean;   /* Mahony ax ← BMI088 accelX */
          float ay0 = ay_mean;   /* Mahony ay ← BMI088 accelY */
          float az0 = az_mean;

          /* 直接由重力方向计算 roll/pitch 安装偏移
           * 注意：运行段 pitch/roll 已对调，此处索引也对调，保持一致 */
          euler_offset[1] = atan2f(ay0, az0) * rad_to_angle;   /* 对调：→ pitch偏移 */
          float sin_p0 = -ax0;
          if      (sin_p0 >  1.0f) sin_p0 =  1.0f;
          else if (sin_p0 < -1.0f) sin_p0 = -1.0f;
          euler_offset[0] = -asinf(sin_p0) * rad_to_angle;       /* 对调：→ roll偏移 */
          euler_offset[2] = 0.0f;                                /* yaw 无绝对参考，归零 */

          /* 用安装偏移初始化 Mahony 四元数，保证后续积分起点正确
           * euler_offset 索引已对调，此处 roll←[1]，pitch←[0] */
          {
            float roll_r  = euler_offset[1] / rad_to_angle;
            float pitch_r = euler_offset[0] / rad_to_angle;
            float cr = cosf(roll_r  * 0.5f);
            float sr = sinf(roll_r  * 0.5f);
            float cp = cosf(pitch_r * 0.5f);
            float sp = sinf(pitch_r * 0.5f);
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
       *          先 AKF 滤除随机噪声，再减静态零偏                      */
      float gx_akf = Gyro_AKF_Update(&akf_gx, gyro[0]);
      float gy_akf = Gyro_AKF_Update(&akf_gy, gyro[1]);
      float gz_akf = gyro[2];   /* gz 不经 AKF，直接透传，避免动态压缩 */

      dbg_gx_akf = gx_akf;
      dbg_gy_akf = gy_akf;
      dbg_gz_akf = gz_akf;

      /* ---- 3b. 减零偏（不做轴交换，与加速度计保持一致）*/
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
#define YAW_STATIC_THR  0.03f   // 静止判断阈值
#define YAW_ZUPT_KP     0.02f
#define YAW_ZUPT_KI     0.001f
#define YAW_BIAS_MAX    0.05f

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

      /* ---- 3e. 四元数 → 欧拉角（ZYX，°，-180~+180）减安装偏移 ---- */
      float roll_m, pitch_m, yaw_m;
      {
        float sin_p = 2.0f * (q[0]*q[2] - q[3]*q[1]);
        if      (sin_p >  1.0f) sin_p =  1.0f;
        else if (sin_p < -1.0f) sin_p = -1.0f;

        roll_m  = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),
                         1.0f-2.0f*(q[1]*q[1]+q[2]*q[2])) * rad_to_angle;
        pitch_m = asinf(sin_p) * rad_to_angle;
        yaw_m   = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]),
                         1.0f-2.0f*(q[2]*q[2]+q[3]*q[3])) * rad_to_angle;

        /* 减安装偏移 */
        roll_m  -= euler_offset[0];
        pitch_m -= euler_offset[1];
        yaw_m   -= euler_offset[2];

        /* pitch/roll 物理轴互换（安装方向决定）*/
        float tmp = roll_m;
        roll_m  = pitch_m;
        pitch_m = -tmp;

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
      dbg_yaw          = yaw_m;   /* IMU Mahony 解算 yaw（°） */

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
      dbg_angle2  = enc_to_angle(motor_info[MOTOR2_IDX].rotor_angle, zero_enc2);
      dbg_angle4  = enc_to_angle(motor_info[MOTOR4_IDX].rotor_angle, zero_enc4);
      dbg_yaw_enc = dbg_angle2;   /* yaw 编码器反馈，供 Ozone 观测及 USB 上报 */

      float spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
      float spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;

      filtered_spd2 = LPF_ALPHA * spd2 + (1.0f - LPF_ALPHA) * filtered_spd2;
      filtered_spd4 = LPF_ALPHA * spd4 + (1.0f - LPF_ALPHA) * filtered_spd4;

      /* ================================================================
       * 4a. 主状态机
       *
       *   STATE_HOMING    → 上电归零（prof_pos 缓慢移向 0°）
       *   STATE_SCAN      → 哨兵扫描（正弦轨迹）
       *   STATE_RETURNING → 自瞄丢目标后回零，到位后切扫描
       *   STATE_TRACK     → 自瞄跟随（视觉绝对角度 + 限位保护）
       *
       *   切换规则（优先级从高到低）：
       *     1. 任意状态：detected=1 且两轴都已完成归位 → STATE_TRACK
       *     2. STATE_TRACK：detected=0 超过 LOST_TIMEOUT_MS → STATE_RETURNING
       *     3. STATE_RETURNING：两轴均到零点（< RETURN_THRESH）→ STATE_SCAN
       *     4. STATE_HOMING：两轴均到零点 → STATE_SCAN（原有逻辑保留）
       * ================================================================ */

      /* ---- 4a-1. 归位阶段（STATE_HOMING）---- */
      if (state2 == STATE_HOMING)
      {
        /* ID2 yaw 轴：homing_target2 独立步进向 0° */
        float step2 = HOMING_SPEED * dt;
        float diff2 = angle_err_calc(0.0f, homing_target2);
        if      (diff2 >  step2) homing_target2 += step2;
        else if (diff2 < -step2) homing_target2 -= step2;
        else                     homing_target2  = 0.0f;

        if (fabsf(angle_err_calc(0.0f, dbg_angle2)) < HOMING_THRESH)
          state2 = STATE_SCAN;
      }

      if (state4 == STATE_HOMING)
      {
        /* ID4 pitch 轴：homing_target4 独立步进向 0°，与 ID2 完全解耦 */
        float step4 = HOMING_SPEED * dt;
        float diff4 = angle_err_calc(0.0f, homing_target4);
        if      (diff4 >  step4) homing_target4 += step4;
        else if (diff4 < -step4) homing_target4 -= step4;
        else                     homing_target4  = 0.0f;

        if (fabsf(angle_err_calc(0.0f, dbg_angle4)) < HOMING_THRESH)
          state4 = STATE_SCAN;
      }

      /* 归位未完成时跳过后续模式切换，防止提前进入自瞄 */
      int homing_done = (state2 != STATE_HOMING && state4 != STATE_HOMING);

      /* ---- 4a-2. 视觉帧活性检测 ----------------------------------------
       *
       *  g_vision_rx.detected 收到帧后不会自动清零，视觉断连时仍保持 1。
       *  用 vis_rx_ok 计数器判断本周期是否有新帧到达：
       *    - 有新帧：vision_active = g_vision_rx.detected（尊重视觉判断）
       *    - 无新帧：vision_active = 0（强制认为丢失，触发超时回零）
       * ------------------------------------------------------------------- */
      static uint32_t last_vis_rx_ok = 0;
      uint8_t vision_active = 0;
      if (vis_rx_ok != last_vis_rx_ok)
      {
        /* 本周期收到了新帧，以视觉 detected 字段为准 */
        vision_active  = g_vision_rx.detected;
        last_vis_rx_ok = vis_rx_ok;
      }
      /* vis_rx_ok 没变：本周期无新帧，vision_active 保持 0 */

      /* ---- 4a-3. 收到视觉数据 → 更新目标角度并切/保持自瞄 ----
       *
       *  vision_active=1 时更新目标，进入 STATE_TRACK。
       *  vision_active=0（无新帧或 detected=0）时开始丢目标计时，
       *  超过 LOST_TIMEOUT_MS 后切 STATE_RETURNING，平滑回零后恢复哨兵扫描。
       * --------------------------------------------------------- */
      if (homing_done && vision_active)
      {
        /* 识别到目标：重置丢目标计时器 */
        lost_timer_ms = 0;

        if (state2 != STATE_TRACK)
        {
          /* 首次进入自瞄：以当前编码器位置为目标起点，清前馈和 PID 积分 */
          track_target_yaw   = dbg_angle2;
          track_target_pitch = dbg_angle4;
          dbg_spd_ff = 0.0f;
          angle_pid2.i_out = 0.0f;
          angle_pid4.i_out = 0.0f;
          speed_pid2.i_out = 0.0f;
          speed_pid4.i_out = 0.0f;
          state2 = STATE_TRACK;
          state4 = STATE_TRACK;
        }
        else
        {
          /* 视觉下发增量（度），叠加到当前目标位置，施加限位 clamp */
          track_target_yaw   += g_vision_rx.yaw_rad;
          track_target_pitch += g_vision_rx.pitch_rad;

          if      (track_target_yaw   >  TRACK_LIMIT_YAW)   track_target_yaw   =  TRACK_LIMIT_YAW;
          else if (track_target_yaw   < -TRACK_LIMIT_YAW)   track_target_yaw   = -TRACK_LIMIT_YAW;
          if      (track_target_pitch >  TRACK_LIMIT_PITCH)  track_target_pitch =  TRACK_LIMIT_PITCH;
          else if (track_target_pitch < -TRACK_LIMIT_PITCH)  track_target_pitch = -TRACK_LIMIT_PITCH;
        }
      }
      else if (state2 == STATE_TRACK)
      {
        /* 自瞄中丢目标（无新帧或 detected=0）：计时，超过 LOST_TIMEOUT_MS 后切回零 */
        lost_timer_ms += 10;  /* 10ms 控制周期 */
        if (lost_timer_ms >= LOST_TIMEOUT_MS)
        {
          /* 从当前编码器位置出发，平滑步进回零 */
          homing_target2 = dbg_angle2;
          homing_target4 = dbg_angle4;
          state2 = STATE_RETURNING;
          state4 = STATE_RETURNING;
          lost_timer_ms = 0;
        }
      }

      /* ---- 4a-4. 回零阶段（STATE_RETURNING）：两轴各自步进向扫描零点 ----
       *
       *  homing_target2 / homing_target4 从自瞄末位置出发，
       *  每帧以 HOMING_SPEED（°/s）独立向 0 步进，互不干扰。
       *  到位条件：各自虚拟目标已到零 且 编码器误差 < RETURN_THRESH。
       *  两轴均满足才切扫描，保证同步完成后再开始正弦轨迹。
       * --------------------------------------------------------------------- */
      if (state2 == STATE_RETURNING)
      {
        float step2 = HOMING_SPEED * dt;
        if      (homing_target2 >  step2) homing_target2 -= step2;
        else if (homing_target2 < -step2) homing_target2 += step2;
        else                              homing_target2  = 0.0f;
      }

      if (state4 == STATE_RETURNING)
      {
        float step4 = HOMING_SPEED * dt;
        if      (homing_target4 >  step4) homing_target4 -= step4;
        else if (homing_target4 < -step4) homing_target4 += step4;
        else                              homing_target4  = 0.0f;
      }

      /* 两轴均完成回零才切扫描 */
      if ((state2 == STATE_RETURNING || state4 == STATE_RETURNING))
      {
        int ret2_done = (state2 != STATE_RETURNING) ||
                        (fabsf(homing_target2) < 0.1f &&
                         fabsf(angle_err_calc(0.0f, dbg_angle2)) < RETURN_THRESH);
        int ret4_done = (state4 != STATE_RETURNING) ||
                        (fabsf(homing_target4) < 0.1f &&
                         fabsf(angle_err_calc(0.0f, dbg_angle4)) < RETURN_THRESH);

        if (ret2_done && ret4_done)
        {
          state2     = STATE_SCAN;
          state4     = STATE_SCAN;
          scan_phase = 0.0f;   /* 相位归零，从 sin(0)=0 重新开始，无位置跳变 */
          prof_pos   = 0.0f;
          homing_target2 = 0.0f;
          homing_target4 = 0.0f;
        }
      }

      /* ---- 4a-5. 扫描阶段：更新正弦轨迹 ---- */
      if (state2 == STATE_SCAN)
      {
        scan_phase += dt;
        float omega    = 2.0f * (float)M_PI * SCAN_FREQ;
        prof_pos       = SCAN_AMP * sinf(omega * scan_phase);
        float ff_deg_s = SCAN_AMP * omega * cosf(omega * scan_phase);
        dbg_spd_ff     = ff_deg_s * SPEED_FF_GAIN;
      }

      /* ---- 4a-6. Ozone 调试变量 ---- */
      /* dbg_state：0=归位  1=扫描  2=回零  3=自瞄 */
      if      (state2 == STATE_HOMING)    dbg_state = 0;
      else if (state2 == STATE_SCAN)      dbg_state = 1;
      else if (state2 == STATE_RETURNING) dbg_state = 2;
      else                                dbg_state = 3;

      dbg_prof = prof_pos;

      /* ================================================================
       * 4b. PID 计算
       *
       *   STATE_TRACK               目标 = 视觉绝对角度（已限位）
       *   STATE_HOMING/RETURNING    目标 = 各轴独立 homing_target（解耦归零）
       *   STATE_SCAN                目标 = prof_pos（正弦扫描轨迹）
       *
       *   ID2（yaw 轴）  ：target = -track_target_yaw / -homing_target2 / -prof_pos
       *   ID4（pitch 轴）：target = -track_target_pitch / -homing_target4 / -prof_pos
       *   负号含义与原始代码一致（电机安装方向取反）
       * ================================================================ */

      /* ---- ID2 PID（yaw 轴）---- */
      {
        float tgt2;
        if      (state2 == STATE_TRACK)     tgt2 = -track_target_yaw;
        else if (state2 == STATE_SCAN)      tgt2 = -prof_pos;
        else /* HOMING / RETURNING */       tgt2 = -homing_target2;

        dbg_err2          = angle_err_calc(tgt2, dbg_angle2);
        angle_pid2.err[1] = angle_pid2.err[0];
        angle_pid2.err[0] = dbg_err2;
        angle_pid2.p_out  = angle_pid2.kp * angle_pid2.err[0];
        angle_pid2.i_out += angle_pid2.ki * angle_pid2.err[0];
        LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
        angle_pid2.d_out  = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
        float pid_spd2    = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;

        /* 自瞄时不叠加扫描前馈 */
        float ff2         = (state2 == STATE_TRACK) ? 0.0f : dbg_spd_ff;
        float spd_target2 = pid_spd2 + ff2;
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

      /* ---- ID4 PID（pitch 轴）---- */
      {
        float tgt4;
        if      (state4 == STATE_TRACK)     tgt4 = -track_target_pitch;
        else if (state4 == STATE_SCAN)      tgt4 = -prof_pos;
        else /* HOMING / RETURNING */       tgt4 = -homing_target4;

        dbg_err4          = angle_err_calc(tgt4, dbg_angle4);
        angle_pid4.err[1] = angle_pid4.err[0];
        angle_pid4.err[0] = dbg_err4;
        angle_pid4.p_out  = angle_pid4.kp * angle_pid4.err[0];
        angle_pid4.i_out += angle_pid4.ki * angle_pid4.err[0];
        LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);
        angle_pid4.d_out  = angle_pid4.kd * (angle_pid4.err[0] - angle_pid4.err[1]);
        float pid_spd4    = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;

        /* 自瞄时不叠加扫描前馈 */
        float ff4         = (state4 == STATE_TRACK) ? 0.0f : dbg_spd_ff;
        float spd_target4 = pid_spd4 + ff4;
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

      set_motor_voltage(0,
                        0,
                        (int16_t)dbg_voltage2,
                        0,
                        (int16_t)dbg_voltage4);

      /* ---------------------------------------------------------------- */
      /* 5. USB 上报当前状态帧（每 10ms 随控制周期发送）                   */
      /*    pitch_deg：IMU Mahony 解算 pitch（度）                         */
      /*    yaw_deg  ：编码器 yaw 角（度），即 dbg_yaw_enc                 */
      /*    mode     ：0x01=扫描/回零/归位  0x02=自瞄                     */
      /* ---------------------------------------------------------------- */
      {
        uint8_t tx_mode = (state2 == STATE_TRACK) ? 0x02 : 0x01;
        BSP_USB_Send(pitch_m, dbg_yaw_enc, tx_mode);  /* yaw 改为编码器反馈（度）*/
      }

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