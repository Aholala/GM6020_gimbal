#ifndef __APP_H
#define __APP_H

#include <stdint.h>
#include "lib_akf.h"

/* ===== 粒子滤波配置 ================================================== */
#define PF_PARTICLE_NUM  25    /* 粒子总数 */
#define PF_STATE_DIM     2     /* 状态维度：[Pitch, Roll] */
#define PF_MEASURE_DIM   2     /* 观测维度：[Pitch, Roll] */

#define PI        3.14159265358979323846f
#define TWO_PI    (2.0f * PI)
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

/* 轴索引 */
#define AXIS_PITCH 0
#define AXIS_ROLL  1

/* ===== AKF 输出缓存（供 static_compensation 使用） =================== */
typedef struct
{
    float gyro_x_hat;
    float gyro_y_hat;
    float gyro_z_hat;
} AKF_Data;

/* ===== 零偏补偿结构体 ================================================= */
typedef struct
{
    float gyro_x_SC;       /* 补偿后 X 轴角速度（最终输出） */
    float gyro_y_SC;       /* 补偿后 Y 轴角速度 */
    float gyro_z_SC;       /* 补偿后 Z 轴角速度 */
    float gyro_x_bias;     /* X 轴常值零偏（校准后固定） */
    float gyro_y_bias;     /* Y 轴常值零偏 */
    float gyro_z_bias;     /* Z 轴常值零偏 */
    uint8_t is_calibrated; /* 0=未校准，1=已校准 */
} SC_Data;

/* ===== MPU 原始数据（仅供 static_compensation 接口兼容） ============== */
typedef struct
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
} MPU_Data;

/* ===== 增强版 Mahony 结构体（BMI088 工程不使用，保留供参考） ========== */
typedef struct {
    float q0, q1, q2, q3;
    float exInt, eyInt, ezInt;
    float delta_acc_smooth;
    float gyro_energy_smooth;
    float alpha_smooth;
    float yaw_bias;
    uint8_t yaw_ref_locked;
    float yaw_static_ref;
    uint8_t yaw_locked;
} Mahony_AHRS_t;

/* ===== 粒子滤波核心结构体 ============================================= */
typedef struct {
    float particles[PF_PARTICLE_NUM][PF_STATE_DIM];
    float weights[PF_PARTICLE_NUM];
    float process_noise_std[PF_STATE_DIM];
    float measure_noise_std[PF_STATE_DIM];
    float state_est_rad[PF_STATE_DIM];
    float state_est_deg[PF_STATE_DIM];
} ParticleFilter_t;

/* ===== 陀螺仪刻度/非正交标定结构体 =================================== */
typedef struct {
    float K[3][3];
} Gyro_Calib_t;

/* ===== 函数声明 ======================================================= */

/* SC 补偿（零偏 + AKF，MPU6050 用，BMI088 工程直接在 main.c 内联） */
void SC_Data_Init(SC_Data *SCData);
void static_compensation(MPU_Data *MPU_Data, AKF_Data *gyro_AKF, SC_Data *SCData,
                         Gyro_AKF_HandleTypeDef *AKF_X,
                         Gyro_AKF_HandleTypeDef *AKF_Y,
                         Gyro_AKF_HandleTypeDef *AKF_Z);

/* 增强版 Mahony（BMI088 工程不使用，保留编译） */
void    Mahony_Init(Mahony_AHRS_t *ahrs);
void    Mahony_Update(Mahony_AHRS_t *ahrs,
                      float ax, float ay, float az,
                      float wx_deg, float wy_deg, float wz_deg,
                      uint8_t is_static, float dt);
void    Mahony_Get_Euler_Rad(Mahony_AHRS_t *ahrs,
                             float *roll_rad, float *pitch_rad, float *yaw_rad);
void    Mahony_Get_Euler_Deg(Mahony_AHRS_t *ahrs,
                             float *roll_deg, float *pitch_deg, float *yaw_deg);
uint8_t Is_Really_Static(float gx_sc, float gy_sc, float gz_sc);
void    Mahony_Init_With_Calibration(Mahony_AHRS_t *ahrs, float ax, float ay, float az);
float   Mahony_Calc_Alpha(Mahony_AHRS_t *ahrs,
                          float ax, float ay, float az,
                          float wx_deg, float wy_deg, float wz_deg);

/* 粒子滤波接口（BMI088 工程使用） */
void  pf_init(ParticleFilter_t *pf, float init_pitch_deg, float init_roll_deg);
void  pf_predict(ParticleFilter_t *pf);
void  pf_update(ParticleFilter_t *pf, float fused_pitch_deg, float fused_roll_deg);
float pf_calculate_neff(ParticleFilter_t *pf);
void  pf_resample(ParticleFilter_t *pf);
void  pf_estimate(ParticleFilter_t *pf);

/* 刻度/非正交补偿 */
void Gyro_Calib_Init(Gyro_Calib_t *calib);
void Gyro_Apply_Scale_Nonortho(Gyro_Calib_t *calib,
                                float raw_gx, float raw_gy, float raw_gz,
                                float *out_gx, float *out_gy, float *out_gz);

#endif