/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 针对 +40度抖动优化的完整版本
  * 集成：动态P增益、增强滤波(0.1)、积分分离与死区
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

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern moto_info_t motor_info[MOTOR_MAX_NUM];

#define MOTOR2_IDX  2
#define MOTOR4_IDX  4

/* ===== PID 基础参数（已根据抖动情况优化） ===== */
#define ANGLE_KP        10.0f
#define ANGLE_KI        0.05f
#define ANGLE_KD        0.4f
#define ANGLE_I_MAX    40.0f
#define ANGLE_OUT_MAX 250.0f

#define SPEED_KP      80.0f    /* 适当降低速度环 P 增益以缓解抖动 */
#define SPEED_KI        0.5f
#define SPEED_KD        0.0f    /* 速度环 D 保持为 0，防止放大噪声 */
#define SPEED_I_MAX  4000.0f
#define SPEED_OUT_MAX 25000.0f

/* ===== 抖动优化专项参数 ===== */
#define LPF_ALPHA       0.1f   /* 增强滤波：从 0.3 降至 0.1，滤除反馈噪声 */
#define ANGLE_DEADBAND  0.1f   /* 位置死区 */
#define I_SEPARATE_THR  2.0f   /* 积分分离阈值 */
#define I_DEADBAND      0.05f  /* 积分死区 */

#define P_REDUCE_THR    2.0f   /* 误差小于 2.0° 时触发 P 增益减小 */
#define P_REDUCE_RATIO  0.5f   /* 增益削减比例，此处为削弱 50% */

/* 其他系统参数保持不变 */
#define GYRO_CALIB_MS    2000
#define ACCEL_VALID_MIN   5.0f
#define ACCEL_VALID_MAX  15.0f
#define AKF_P0           1.0f
#define AKF_Q0           1e-7f
#define PF_NEFF_THRESH  (PF_PARTICLE_NUM / 2.0f)
#define SCAN_AMP        40.0f
#define SCAN_FREQ        0.5f
#define SPEED_FF_GAIN    0.0f
#define HOMING_SPEED    20.0f
#define HOMING_THRESH    3.0f

pid_struct_t angle_pid2, angle_pid4;
pid_struct_t speed_pid2, speed_pid4;

typedef enum { STATE_HOMING = 0, STATE_SCAN } ctrl_state_t;
ctrl_state_t state2 = STATE_HOMING;
ctrl_state_t state4 = STATE_HOMING;

float prof_pos = 0.0f;
float scan_phase = 0.0f;
float dbg_spd_ff = 0.0f;
float filtered_spd2 = 0.0f;
float filtered_spd4 = 0.0f;
uint16_t zero_enc2 = 0;
uint16_t zero_enc4 = 0;

/* Ozone 观测变量 */
volatile float dbg_angle2, dbg_angle4, dbg_prof, dbg_err2, dbg_err4;
volatile float dbg_voltage2, dbg_voltage4;
volatile int imu_calib_done = 0;
float gyro_bias[3] = {0}, euler_offset[3] = {0};
Gyro_AKF_HandleTypeDef akf_gx, akf_gy, akf_gz;
ParticleFilter_t pf;
/* USER CODE END PM */

/* USER CODE BEGIN 0 */
float enc_to_angle(uint16_t enc, uint16_t zero) {
    int32_t diff = (int32_t)enc - (int32_t)zero;
    if (diff > 4096) diff -= 8192; else if (diff < -4096) diff += 8192;
    return (float)diff * 360.0f / 8192.0f;
}

float angle_err_calc(float target, float current) {
    float e = target - current;
    if (e > 180.0f) e -= 360.0f; else if (e < -180.0f) e += 360.0f;
    return e;
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    SystemClock_Config();
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

    BMI088_init();
    HAL_Delay(100);
    zero_enc2 = motor_info[MOTOR2_IDX].rotor_angle;
    zero_enc4 = motor_info[MOTOR4_IDX].rotor_angle;
    BSP_USB_Init();
    /* USER CODE END 2 */

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    while (1) {
        /* USER CODE BEGIN 3 */
        BSP_USB_Receive();
        static uint32_t last_tick = 0;
        if (HAL_GetTick() - last_tick >= 10) {
            last_tick = HAL_GetTick();
            const float dt = 0.01f;

            fp32 gyro[3], accel[3], temp;
            BMI088_read(gyro, accel, &temp);

            /* --- IMU 校准逻辑 --- */
            if (!imu_calib_done) {
                static int calib_cnt = 0;
                static double g_sum[3] = {0};
                g_sum[0]+=gyro[0]; g_sum[1]+=gyro[1]; g_sum[2]+=gyro[2];
                if (++calib_cnt >= 200) {
                    gyro_bias[0]=g_sum[0]/200; gyro_bias[1]=g_sum[1]/200; gyro_bias[2]=g_sum[2]/200;
                    Gyro_AKF_Init(&akf_gx, gyro_bias[0], AKF_P0, AKF_Q0, 1e-4f);
                    Gyro_AKF_Init(&akf_gy, gyro_bias[1], AKF_P0, AKF_Q0, 1e-4f);
                    Gyro_AKF_Init(&akf_gz, gyro_bias[2], AKF_P0, AKF_Q0, 1e-4f);
                    imu_calib_done = 1;
                }
                continue;
            }

            /* --- 姿态融合更新 --- */
            float gx = Gyro_AKF_Update(&akf_gx, gyro[0]) - gyro_bias[0];
            float gy = Gyro_AKF_Update(&akf_gy, gyro[1]) - gyro_bias[1];
            float gz = Gyro_AKF_Update(&akf_gz, gyro[2]) - gyro_bias[2];
            MahonyAHRSupdateIMU(q, gx, gy, gz, accel[1], accel[0], accel[2]);

            /* --- 反馈值读取与滤波 --- */
            dbg_angle2 = enc_to_angle(motor_info[MOTOR2_IDX].rotor_angle, zero_enc2);
            dbg_angle4 = enc_to_angle(motor_info[MOTOR4_IDX].rotor_angle, zero_enc4);

            // 增强速度反馈低通滤波
            filtered_spd2 = LPF_ALPHA * (float)motor_info[MOTOR2_IDX].rotor_speed + (1.0f - LPF_ALPHA) * filtered_spd2;
            filtered_spd4 = LPF_ALPHA * (float)motor_info[MOTOR4_IDX].rotor_speed + (1.0f - LPF_ALPHA) * filtered_spd4;

            // 正弦扫描目标生成
            scan_phase += dt;
            prof_pos = SCAN_AMP * sinf(2.0f * (float)M_PI * SCAN_FREQ * scan_phase);
            dbg_prof = prof_pos;

            /* ---- ID2 PID 优化核心 ---- */
            {
                dbg_err2 = angle_err_calc(-prof_pos, dbg_angle2);

                // 1. 位置环全局死区
                if (fabsf(dbg_err2) < ANGLE_DEADBAND) dbg_err2 = 0.0f;

                // 2. 动态调节 KP：误差越小，P越弱，抑制抖动
                float dynamic_kp = ANGLE_KP;
                if (fabsf(dbg_err2) < P_REDUCE_THR) {
                    dynamic_kp = ANGLE_KP * P_REDUCE_RATIO;
                }
                angle_pid2.p_out = dynamic_kp * dbg_err2;

                // 3. 积分优化：死区外积分，死区内平滑衰减
                if (fabsf(dbg_err2) > I_DEADBAND && fabsf(dbg_err2) < I_SEPARATE_THR) {
                    angle_pid2.i_out += angle_pid2.ki * dbg_err2;
                } else if (fabsf(dbg_err2) <= I_DEADBAND) {
                    angle_pid2.i_out *= 0.8f;
                }
                LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);

                // 4. 微分项 D
                angle_pid2.d_out = angle_pid2.kd * (dbg_err2 - angle_pid2.err[0]);
                angle_pid2.err[0] = dbg_err2;

                float spd_target2 = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;
                LIMIT_MIN_MAX(spd_target2, -angle_pid2.out_max, angle_pid2.out_max);

                // 5. 速度环计算
                float spd_err2 = spd_target2 - filtered_spd2;
                speed_pid2.p_out = speed_pid2.kp * spd_err2;
                speed_pid2.i_out += speed_pid2.ki * spd_err2;
                LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);

                dbg_voltage2 = speed_pid2.p_out + speed_pid2.i_out;
                LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);
            }

            /* ---- ID4 PID 优化核心 ---- */
            {
                dbg_err4 = angle_err_calc(-prof_pos, dbg_angle4);
                if (fabsf(dbg_err4) < ANGLE_DEADBAND) dbg_err4 = 0.0f;

                float dynamic_kp4 = (fabsf(dbg_err4) < P_REDUCE_THR) ? (ANGLE_KP * P_REDUCE_RATIO) : ANGLE_KP;
                angle_pid4.p_out = dynamic_kp4 * dbg_err4;

                if (fabsf(dbg_err4) > I_DEADBAND && fabsf(dbg_err4) < I_SEPARATE_THR) {
                    angle_pid4.i_out += angle_pid4.ki * dbg_err4;
                } else if (fabsf(dbg_err4) <= I_DEADBAND) {
                    angle_pid4.i_out *= 0.8f;
                }
                LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);

                angle_pid4.d_out = angle_pid4.kd * (dbg_err4 - angle_pid4.err[0]);
                angle_pid4.err[0] = dbg_err4;

                float spd_target4 = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;
                LIMIT_MIN_MAX(spd_target4, -angle_pid4.out_max, angle_pid4.out_max);

                float spd_err4 = spd_target4 - filtered_spd4;
                speed_pid4.p_out = speed_pid4.kp * spd_err4;
                speed_pid4.i_out += speed_pid4.ki * spd_err4;
                LIMIT_MIN_MAX(speed_pid4.i_out, -speed_pid4.i_max, speed_pid4.i_max);

                dbg_voltage4 = speed_pid4.p_out + speed_pid4.i_out;
                LIMIT_MIN_MAX(dbg_voltage4, -speed_pid4.out_max, speed_pid4.out_max);
            }

            // 输出电流控制
            set_motor_voltage(0, 0, (int16_t)dbg_voltage2, 0, (int16_t)dbg_voltage4);
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
  * where the assert_param error has occurred.
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