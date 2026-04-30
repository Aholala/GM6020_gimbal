#include "math.h"
#include "AKF.h"
#include "Config.h"
#include <stdlib.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/***************************角度单位换算********************************/

#define DEG2RAD  0.017453292519943295f  // 度转弧度
#define RAD2DEG  57.29577951308232f     // 弧度转度
/*********************************************************************/

// 静态判断：角速度波动小于阈值
#define STABILITY_THRESHOLD  0.001f   // 角速度变化阈值（rad/s），小于此值认为静止
#define STABILITY_COUNT      100     // 静止计数器阈值（1ms判断一次，100ms即认为静止）

/************************** Mahony滤波器参数 **************************/
// Mahony自适应参数
#define KP_STATIC  4.0f    // 静止时的比例增益
#define KP_DYNAMIC 0.01f   // 运动时的比例增益
#define KI_STATIC  0.002f  // 静止时的积分增益
#define KI_DYNAMIC 0.0f    // 运动时的积分增益
#define INTEGRAL_MAX_LIMIT  0.05f  // 积分项上限（rad/s）
#define INTEGRAL_MIN_LIMIT -0.05f  // 积分项下限

// 陀螺仪静态检测阈值及迟滞（用于ZUPT）
#define GYRO_STATIC_THRESHOLD  2.0f  // 静态角速度阈值（deg/s）
#define GYRO_HYSTERESIS        0.5f  // 迟滞量

/************************** 梯度补偿功能开关 **************************/
#define ENABLE_GRADIENT_COMPENSATE  1  // 1=使能梯度补偿，0=禁止
#define COMPENSATE_INTERVAL         10  // 每10ms执行一次梯度下降补偿
/************************** 梯度补偿安全参数 **************************/
#define COMPENSATE_STEP_SAFE       0.0001f // 补偿步长（安全小值）
#define BIAS_MAX_LIMIT_SAFE        1.0f     // 偏置上限
#define BIAS_MIN_LIMIT_SAFE       -1.0f     // 偏置下限
/**************************************************************************/

/* IMU_Calibrate_Accel_Offset: MPU6050 专用，BMI088 工程不使用，已注释 */

/**
 * @brief  静态校准加速度计ax、ay，采样100次取平均
 * @param  ax_offset: 加速度计x轴零偏输出指针
 * @param  ay_offset: 加速度计y轴零偏输出指针
 * @note   1. 调用前需确保IMU已通过I2C/SPI正常通信
 * @note   2. 校准过程中IMU应保持绝对静止
 * @note   3. 校准完成后零偏值会被用于后续姿态解算
 */

void SC_Data_Init(SC_Data* SCData)
{
    SCData->gyro_x_SC = 0.0f;
    SCData->gyro_y_SC = 0.0f;
    SCData->gyro_z_SC = 0.0f;
    SCData->gyro_x_bias = 0.0f;
    SCData->gyro_y_bias = 0.0f;
    SCData->gyro_z_bias = 0.0f;
    SCData->is_calibrated = 0;  // 默认未校准
}

void static_compensation(MPU_Data* MPU_Data, AKF_Data* gyro_AKF, SC_Data* SCData, Gyro_AKF_HandleTypeDef* AKF_X, Gyro_AKF_HandleTypeDef* AKF_Y, Gyro_AKF_HandleTypeDef* AKF_Z)
{
    // 静态补偿：利用AKF滤波后的陀螺数据，在静止时计算零偏
    static float prev_gx = 0.0f, prev_gy = 0.0f, prev_gz = 0.0f;
    static uint16_t stable_cnt = 0;

    // 先对原始陀螺数据进行AKF滤波
    gyro_AKF->gyro_x_hat = Gyro_AKF_Update(AKF_X, MPU_Data->gx);
    gyro_AKF->gyro_y_hat = Gyro_AKF_Update(AKF_Y, MPU_Data->gy);
    gyro_AKF->gyro_z_hat = Gyro_AKF_Update(AKF_Z, MPU_Data->gz);

    // 零偏校准逻辑
    if (SCData->is_calibrated == 0)
    {
        // ====================== 未校准：判断是否静止 ======================
        float dx = fabs(gyro_AKF->gyro_x_hat - prev_gx);
        float dy = fabs(gyro_AKF->gyro_y_hat - prev_gy);
        float dz = fabs(gyro_AKF->gyro_z_hat - prev_gz);

        // 更新前一时刻值
        prev_gx = gyro_AKF->gyro_x_hat;
        prev_gy = gyro_AKF->gyro_y_hat;
        prev_gz = gyro_AKF->gyro_z_hat;

        // 判断三个轴是否同时稳定
        if (dx < STABILITY_THRESHOLD && dy < STABILITY_THRESHOLD && dz < STABILITY_THRESHOLD)
        {
            stable_cnt++;
            if (stable_cnt >= STABILITY_COUNT)
            {
                // 静止足够长时间，记录当前滤波值作为零偏
                SCData->gyro_x_bias = gyro_AKF->gyro_x_hat;
                SCData->gyro_y_bias = gyro_AKF->gyro_y_hat;
                SCData->gyro_z_bias = gyro_AKF->gyro_z_hat;
                SCData->is_calibrated = 1;  // 校准完成，后续不再重复校准
                stable_cnt = 0;               // 重置计数器
            }
        }
        else
        {
            stable_cnt = 0;  // 运动状态，重置计数器
        }

        // 校准完成前，暂用当前值减零偏（零偏初始为0）
        SCData->gyro_x_SC = gyro_AKF->gyro_x_hat - SCData->gyro_x_bias;
        SCData->gyro_y_SC = gyro_AKF->gyro_y_hat - SCData->gyro_y_bias;
        SCData->gyro_z_SC = gyro_AKF->gyro_z_hat - SCData->gyro_z_bias;
    }
    else
    {
        // ====================== 已校准：直接扣除零偏 ======================
        SCData->gyro_x_SC = gyro_AKF->gyro_x_hat - SCData->gyro_x_bias;
        SCData->gyro_y_SC = gyro_AKF->gyro_y_hat - SCData->gyro_y_bias;
        SCData->gyro_z_SC = gyro_AKF->gyro_z_hat - SCData->gyro_z_bias;
    }
}

/**************************************************************************
 * 函数名：Mahony_Init
 * 功能：初始化Mahony姿态解算结构体
 * 参数：ahrs —— 姿态解算结构体指针
 **************************************************************************/
void Mahony_Init(Mahony_AHRS_t *ahrs)
{
    // 1. 初始化四元数为单位四元数
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    // 2. 清零积分误差
    ahrs->exInt = 0.0f;
    ahrs->eyInt = 0.0f;
    ahrs->ezInt = 0.0f;

    // 3. 平滑变量初始化
    ahrs->delta_acc_smooth = 0.0f;
    ahrs->gyro_energy_smooth = 0.0f;
    ahrs->alpha_smooth = 0.0f;

    // 偏航角相关变量归零
    ahrs->yaw_bias = 0.0f;
    ahrs->yaw_static_ref = 0.0f;
    ahrs->yaw_locked = 0;
}

/**************************************************************************
 * 函数名：Is_Really_Static
 * 功能：根据陀螺仪校准后的数据判断IMU是否真正静止（带迟滞）
 * 参数：gx_sc, gy_sc, gz_sc —— 校准后的陀螺仪角速度（deg/s）
 * 返回：1=静止，0=运动
 * 说明：实际使用时传入 SC_Data 中的 gyro_x_SC 等
 **************************************************************************/
uint8_t Is_Really_Static(float gx_sc, float gy_sc, float gz_sc)
{
    static uint8_t is_static_last = 1;

    // 1. 计算当前角速度模值
    float gyro_norm = sqrt(gx_sc*gx_sc + gy_sc*gy_sc + gz_sc*gz_sc);
    uint8_t gyro_static = 0;

    // 2. 迟滞比较
    if (is_static_last) {
        // 上一时刻静止：进入静止需要模值小于（阈值+迟滞）
        if (gyro_norm < (GYRO_STATIC_THRESHOLD + GYRO_HYSTERESIS)) {
            gyro_static = 1;
        }
    } else {
        // 上一时刻运动：进入静止需要模值小于（阈值-迟滞）
        if (gyro_norm < (GYRO_STATIC_THRESHOLD - GYRO_HYSTERESIS)) {
            gyro_static = 1;
        }
    }

    // 3. 仅用陀螺仪判断，不考虑加速度计（因为加速度计易受振动干扰）
    uint8_t is_static = gyro_static;
    is_static_last = is_static;
    return is_static;
}

/**************************************************************************
 * 函数名：Mahony_Init_With_Calibration
 * 功能：利用初始加速度计数据计算Roll/Pitch并初始化四元数（不依赖Yaw）
 * 参数：ahrs —— 姿态解算结构体
 *       ax, ay, az —— 加速度计原始数据（单位：g）
 * 说明：适用于上电静止时调用，可快速获得水平姿态
 **************************************************************************/
void Mahony_Init_With_Calibration(Mahony_AHRS_t *ahrs, float ax, float ay, float az)
{
    // 1. 加速度归一化
    float norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm > 0.0001f) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    // 2. 计算初始Roll/Pitch（弧度）
    float init_roll = atan2(ay, az);
    float init_pitch = -asin(ax);

    // 3. 根据Roll/Pitch构造四元数（忽略Yaw，设Yaw初始为0）
    float cr = cos(init_roll * 0.5f);
    float sr = sin(init_roll * 0.5f);
    float cp = cos(init_pitch * 0.5f);
    float sp = sin(init_pitch * 0.5f);

    ahrs->q0 = cr * cp;
    ahrs->q1 = sr * cp;
    ahrs->q2 = cr * sp;
    ahrs->q3 = -sr * sp;

    // 4. 清零积分误差
    ahrs->exInt = 0.0f;
    ahrs->eyInt = 0.0f;
    ahrs->ezInt = 0.0f;
}

/**************************************************************************
 * 函数名：Mahony_Update
 * 功能：改进版Mahony姿态解算（ZUPT偏航漂移抑制 + RK2四元数积分）
 *       利用静止检测动态调整KP/KI，并对Yaw零偏进行在线估计
 * 参数：ahrs —— 姿态结构体
 *       ax, ay, az —— 加速度计数据（单位：g）
 *       wx_deg, wy_deg, wz_deg —— 陀螺仪角速度（单位：度/秒）
 *       is_static —— 静止标志（1静止，0运动）
 *       dt —— 解算周期（秒）
 **************************************************************************/
void Mahony_Update(Mahony_AHRS_t *ahrs, float ax, float ay, float az, float wx_deg, float wy_deg, float wz_deg, uint8_t is_static, float dt)
{
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // 1. 根据静止状态选择滤波器增益
    float Kp = is_static ? KP_STATIC : KP_DYNAMIC;
    float Ki = is_static ? KI_STATIC : KI_DYNAMIC;

    // 2. 角度单位转换：度/秒 -> 弧度/秒
    float wx = wx_deg * DEG2RAD;
    float wy = wy_deg * DEG2RAD;
    float wz = wz_deg * DEG2RAD;

    // ========== ZUPT零速修正：静止时估计Yaw轴零偏 ==========
    if (is_static)
    {
        // 静止时采用两种速率修正
        const float KP_YAW_SLOW = 0.015f;  // 低速修正系数
        const float KP_YAW_FAST = 0.02f;   // 高速修正系数

        // 去除当前零偏后的Z轴角速度
        float wz_corrected = wz - ahrs->yaw_bias;

        // 根据修正后的角速度大小选择修正速度
        float kp_yaw = (fabsf(wz_corrected) > 0.005f) ? KP_YAW_FAST : KP_YAW_SLOW;

        // 积分方式更新零偏（100%反馈）
        ahrs->yaw_bias += kp_yaw * wz_corrected * dt;

        // 限制零偏最大值 ±0.1rad/s （约±5.7°/s）
        const float YAW_BIAS_MAX = 0.1f;
        ahrs->yaw_bias = (ahrs->yaw_bias > YAW_BIAS_MAX) ? YAW_BIAS_MAX : ahrs->yaw_bias;
        ahrs->yaw_bias = (ahrs->yaw_bias < -YAW_BIAS_MAX) ? -YAW_BIAS_MAX : ahrs->yaw_bias;
    }
    // 运动时/静止时均扣除零偏（使ZUPT效果持续）
    wz -= ahrs->yaw_bias;
    // ========== ZUPT修正结束 ==========

    // 3. 加速度计归一化（用于计算参考向量）
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 0.0001f) {
        ax /= norm; ay /= norm; az /= norm;
    } else {
        return;
    }

    // 4. 根据当前四元数计算重力加速度估计值（参考向量）
    halfvx = ahrs->q1 * ahrs->q3 - ahrs->q0 * ahrs->q2;
    halfvy = ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3;
    halfvz = ahrs->q0 * ahrs->q0 - 0.5f + ahrs->q3 * ahrs->q3;

    // 5. 计算误差（实测加速度与估计加速度的叉积）
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // 6. 积分误差累积（仅在Ki>0时有效）
    if (Ki > 0.0f) {
        ahrs->exInt += Ki * halfex * dt;
        ahrs->eyInt += Ki * halfey * dt;
        ahrs->ezInt += Ki * halfez * dt;

        ahrs->exInt = (ahrs->exInt > INTEGRAL_MAX_LIMIT) ? INTEGRAL_MAX_LIMIT : ahrs->exInt;
        ahrs->exInt = (ahrs->exInt < INTEGRAL_MIN_LIMIT) ? INTEGRAL_MIN_LIMIT : ahrs->exInt;
        ahrs->eyInt = (ahrs->eyInt > INTEGRAL_MAX_LIMIT) ? INTEGRAL_MAX_LIMIT : ahrs->eyInt;
        ahrs->eyInt = (ahrs->eyInt < INTEGRAL_MIN_LIMIT) ? INTEGRAL_MIN_LIMIT : ahrs->eyInt;
        ahrs->ezInt = (ahrs->ezInt > INTEGRAL_MAX_LIMIT) ? INTEGRAL_MAX_LIMIT : ahrs->ezInt;
        ahrs->ezInt = (ahrs->ezInt < INTEGRAL_MIN_LIMIT) ? INTEGRAL_MIN_LIMIT : ahrs->ezInt;

        wx += ahrs->exInt;
        wy += ahrs->eyInt;
        wz += ahrs->ezInt;
    } else {
        // 无积分时清空积分项
        ahrs->exInt = 0.0f;
        ahrs->eyInt = 0.0f;
        ahrs->ezInt = 0.0f;
    }

    // 7. 比例项补偿角速度
    wx += Kp * halfex;
    wy += Kp * halfey;
    wz += Kp * halfez;

    // ========== RK2（二阶龙格-库塔）四元数更新 ==========
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float half_wx = 0.5f * wx;
    float half_wy = 0.5f * wy;
    float half_wz = 0.5f * wz;

    // 第一步：计算k1（dt处的斜率）
    float k1_q0 = -q1 * half_wx - q2 * half_wy - q3 * half_wz;
    float k1_q1 =  q0 * half_wx + q2 * half_wz - q3 * half_wy;
    float k1_q2 =  q0 * half_wy - q1 * half_wz + q3 * half_wx;
    float k1_q3 =  q0 * half_wz + q1 * half_wy - q2 * half_wx;

    // 第二步：计算中点处的四元数值
    float half_dt = 0.5f * dt;
    float q_mid0 = q0 + k1_q0 * half_dt;
    float q_mid1 = q1 + k1_q1 * half_dt;
    float q_mid2 = q2 + k1_q2 * half_dt;
    float q_mid3 = q3 + k1_q3 * half_dt;

    norm = sqrtf(q_mid0*q_mid0 + q_mid1*q_mid1 + q_mid2*q_mid2 + q_mid3*q_mid3);
    if (norm > 0.0001f) {
        q_mid0 /= norm; q_mid1 /= norm; q_mid2 /= norm; q_mid3 /= norm;
    } else {
        return;
    }

    // 第三步：计算k2（中点处的斜率）
    float k2_q0 = -q_mid1 * half_wx - q_mid2 * half_wy - q_mid3 * half_wz;
    float k2_q1 =  q_mid0 * half_wx + q_mid2 * half_wz - q_mid3 * half_wy;
    float k2_q2 =  q_mid0 * half_wy - q_mid1 * half_wz + q_mid3 * half_wx;
    float k2_q3 =  q_mid0 * half_wz + q_mid1 * half_wy - q_mid2 * half_wx;

    // 第四步：用k2更新四元数
    ahrs->q0 = q0 + k2_q0 * dt;
    ahrs->q1 = q1 + k2_q1 * dt;
    ahrs->q2 = q2 + k2_q2 * dt;
    ahrs->q3 = q3 + k2_q3 * dt;

    // 第五步：四元数归一化
    norm = sqrtf(ahrs->q0*ahrs->q0 + ahrs->q1*ahrs->q1 + ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3);
    if (norm > 0.0001f) {
        ahrs->q0 /= norm; ahrs->q1 /= norm; ahrs->q2 /= norm; ahrs->q3 /= norm;
    }
    // ========== RK2积分结束 ==========
}

/**************************************************************************
 * 函数名：Mahony_Get_Euler_Rad
 * 功能：从四元数计算欧拉角（弧度）
 **************************************************************************/
void Mahony_Get_Euler_Rad(Mahony_AHRS_t *ahrs, float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    *roll_rad = atan2(2.0f * (ahrs->q0*ahrs->q1 + ahrs->q2*ahrs->q3), 1.0f - 2.0f * (ahrs->q1*ahrs->q1 + ahrs->q2*ahrs->q2));
    *pitch_rad = asin(2.0f * (ahrs->q0*ahrs->q2 - ahrs->q3*ahrs->q1));
    *yaw_rad = atan2(2.0f * (ahrs->q0*ahrs->q3 + ahrs->q1*ahrs->q2), 1.0f - 2.0f * (ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3));
}

/**************************************************************************
 * 函数名：Mahony_Get_Euler_Deg
 * 功能：从四元数计算欧拉角（度）
 **************************************************************************/
void Mahony_Get_Euler_Deg(Mahony_AHRS_t *ahrs, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll_rad, pitch_rad, yaw_rad;
    Mahony_Get_Euler_Rad(ahrs, &roll_rad, &pitch_rad, &yaw_rad);
    *roll_deg = roll_rad * RAD2DEG;
    *pitch_deg = pitch_rad * RAD2DEG;
    *yaw_deg = yaw_rad * RAD2DEG;
}

// ===================== 辅助函数1：角度归一化到 [-π, π) =====================
static float pf_wrap_angle_rad(float angle_rad) {
    while (angle_rad >= M_PI) {
        angle_rad -= 2.0f * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0f * M_PI;
    }
    return angle_rad;
}

// ===================== 辅助函数2：角度归一化到 [-180°, 180°) =====================
static float pf_wrap_angle_deg(float angle_deg) {
    while (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

// ===================== 辅助函数3：角度度转弧度（并归一化） =====================
static float pf_deg_to_rad(float angle_deg) {
    float normalized_deg = pf_wrap_angle_deg(angle_deg);
    return normalized_deg * DEG2RAD;
}

// ===================== 辅助函数4：弧度角度差（考虑环绕） =====================
static float pf_angle_diff_rad(float angle1_rad, float angle2_rad) {
    float diff = angle1_rad - angle2_rad;
    return pf_wrap_angle_rad(diff);
}

// ===================== 辅助函数5：生成高斯白噪声（Box-Muller） =====================
static float pf_gaussian_rand(float std) {
    static int has_spare = 0;
    static float spare = 0.0f;
    float u1, u2, s;

    if (has_spare) {
        has_spare = 0;
        return spare * std;
    }

    do {
        u1 = (float)rand() / RAND_MAX;
        u2 = (float)rand() / RAND_MAX;
        u1 = 2.0f * u1 - 1.0f;
        u2 = 2.0f * u2 - 1.0f;
        s = u1 * u1 + u2 * u2;
    } while (s >= 1.0f || s == 0.0f);

    s = sqrtf(-2.0f * logf(s) / s);
    spare = u2 * s;
    has_spare = 1;
    return u1 * s * std;
}

// ===================== 粒子滤波器初始化 =====================
// 功能：初始化粒子滤波器，设定初始Pitch/Roll及噪声参数
// 参数：init_pitch_deg - 初始Pitch角度（度）
//       init_roll_deg  - 初始Roll角度（度）
void pf_init(ParticleFilter_t *pf, float init_pitch_deg, float init_roll_deg) {
    // 1. 转换为弧度
    float init_pitch_rad = pf_deg_to_rad(init_pitch_deg);
    float init_roll_rad = pf_deg_to_rad(init_roll_deg);

    // 2. 设定过程噪声和测量噪声标准差（弧度）
    // Pitch轴
    pf->process_noise_std[AXIS_PITCH] = 0.002f;  // 约0.11度
    pf->measure_noise_std[AXIS_PITCH] = 0.006f;  // 约0.34度
    // Roll轴
    pf->process_noise_std[AXIS_ROLL] = 0.002f;   // 约0.11度
    pf->measure_noise_std[AXIS_ROLL] = 0.006f;   // 约0.34度

    // 3. 初始化粒子群（围绕初始值添加高斯扰动）
    float init_std = 0.035f; // 约2度，初始散布范围
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        // Pitch粒子
        float raw_pitch = init_pitch_rad + pf_gaussian_rand(init_std);
        pf->particles[i][AXIS_PITCH] = pf_wrap_angle_rad(raw_pitch);
        // Roll粒子
        float raw_roll = init_roll_rad + pf_gaussian_rand(init_std);
        pf->particles[i][AXIS_ROLL] = pf_wrap_angle_rad(raw_roll);
    }

    // 4. 所有粒子权重相等
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        pf->weights[i] = 1.0f / PF_PARTICLE_NUM;
    }

    // 5. 初始化状态估计值
    pf->state_est_rad[AXIS_PITCH] = init_pitch_rad;
    pf->state_est_rad[AXIS_ROLL] = init_roll_rad;
    pf->state_est_deg[AXIS_PITCH] = init_pitch_deg;
    pf->state_est_deg[AXIS_ROLL] = init_roll_deg;
}

// ===================== 粒子滤波器预测步 =====================
void pf_predict(ParticleFilter_t *pf) {
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        // Pitch加过程噪声
        float noise_pitch = pf_gaussian_rand(pf->process_noise_std[AXIS_PITCH]);
        float raw_pitch = pf->particles[i][AXIS_PITCH] + noise_pitch;
        pf->particles[i][AXIS_PITCH] = pf_wrap_angle_rad(raw_pitch);
        // Roll加过程噪声
        float noise_roll = pf_gaussian_rand(pf->process_noise_std[AXIS_ROLL]);
        float raw_roll = pf->particles[i][AXIS_ROLL] + noise_roll;
        pf->particles[i][AXIS_ROLL] = pf_wrap_angle_rad(raw_roll);
    }
}

// ===================== 粒子滤波器更新步 =====================
// 功能：根据融合后的Pitch/Roll观测值更新粒子权重
// 参数：fused_pitch_deg - 融合后的Pitch角度（度）
//       fused_roll_deg  - 融合后的Roll角度（度）
void pf_update(ParticleFilter_t *pf, float fused_pitch_deg, float fused_roll_deg) {
    // 1. 观测值转为弧度
    float fused_pitch_rad = pf_deg_to_rad(fused_pitch_deg);
    float fused_roll_rad = pf_deg_to_rad(fused_roll_deg);

    // 2. 计算观测噪声协方差（方差）
    float R_pitch = pf->measure_noise_std[AXIS_PITCH] * pf->measure_noise_std[AXIS_PITCH];
    float R_roll = pf->measure_noise_std[AXIS_ROLL] * pf->measure_noise_std[AXIS_ROLL];

    float weight_sum = 0.0f;

    // 3. 计算每个粒子的似然并更新权重
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        // --- Pitch似然 ---
        float predicted_pitch = pf->particles[i][AXIS_PITCH];
        float error_pitch = pf_angle_diff_rad(fused_pitch_rad, predicted_pitch);
        float likelihood_pitch = expf(-0.5f * error_pitch * error_pitch / R_pitch);

        // --- Roll似然 ---
        float predicted_roll = pf->particles[i][AXIS_ROLL];
        float error_roll = pf_angle_diff_rad(fused_roll_rad, predicted_roll);
        float likelihood_roll = expf(-0.5f * error_roll * error_roll / R_roll);

        // --- 联合似然（假设Pitch和Roll独立）---
        float joint_likelihood = likelihood_pitch * likelihood_roll;

        // 更新权重
        pf->weights[i] = pf->weights[i] * joint_likelihood;
        weight_sum += pf->weights[i];
    }

    // 4. 权重归一化
    if (weight_sum < 1e-10f) {
        // 权重退化严重，重置为均匀分布
        for (int i = 0; i < PF_PARTICLE_NUM; i++) {
            pf->weights[i] = 1.0f / PF_PARTICLE_NUM;
        }
    } else {
        for (int i = 0; i < PF_PARTICLE_NUM; i++) {
            pf->weights[i] = pf->weights[i] / weight_sum;
        }
    }
}

// ===================== 计算有效粒子数 =====================
float pf_calculate_neff(ParticleFilter_t *pf) {
    float sum_sq = 0.0f;
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        sum_sq += pf->weights[i] * pf->weights[i];
    }
    return 1.0f / sum_sq;
}

// ===================== 系统重采样（低方差采样） =====================
void pf_resample(ParticleFilter_t *pf) {
    float new_particles[PF_PARTICLE_NUM][PF_STATE_DIM];
    float new_weights[PF_PARTICLE_NUM];

    float step = 1.0f / PF_PARTICLE_NUM;
    float u = ((float)rand() / RAND_MAX) * step;
    float cumulative_sum = pf->weights[0];
    int j = 0;

    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        while (u > cumulative_sum) {
            j++;
            cumulative_sum += pf->weights[j];
        }

        // 复制粒子（并确保角度范围）
        new_particles[i][AXIS_PITCH] = pf_wrap_angle_rad(pf->particles[j][AXIS_PITCH]);
        new_particles[i][AXIS_ROLL] = pf_wrap_angle_rad(pf->particles[j][AXIS_ROLL]);
        new_weights[i] = 1.0f / PF_PARTICLE_NUM;

        u += step;
    }

    // 替换旧粒子群
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        pf->particles[i][AXIS_PITCH] = new_particles[i][AXIS_PITCH];
        pf->particles[i][AXIS_ROLL] = new_particles[i][AXIS_ROLL];
        pf->weights[i] = new_weights[i];
    }
}

// ===================== 状态估计（加权平均，处理循环角度） =====================
void pf_estimate(ParticleFilter_t *pf) {
    // --- Pitch分量统计 ---
    float sum_sin_pitch = 0.0f;
    float sum_cos_pitch = 0.0f;
    // --- Roll分量统计 ---
    float sum_sin_roll = 0.0f;
    float sum_cos_roll = 0.0f;

    // 1. 加权求和sin/cos
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        float w = pf->weights[i];

        // Pitch
        float pitch = pf->particles[i][AXIS_PITCH];
        sum_sin_pitch += w * sinf(pitch);
        sum_cos_pitch += w * cosf(pitch);

        // Roll
        float roll = pf->particles[i][AXIS_ROLL];
        sum_sin_roll += w * sinf(roll);
        sum_cos_roll += w * cosf(roll);
    }

    // 2. 计算平均角度（atan2）
    pf->state_est_rad[AXIS_PITCH] = atan2f(sum_sin_pitch, sum_cos_pitch);
    pf->state_est_rad[AXIS_ROLL] = atan2f(sum_sin_roll, sum_cos_roll);

    // 3. 转换为度数输出
    pf->state_est_deg[AXIS_PITCH] = pf->state_est_rad[AXIS_PITCH] * RAD2DEG;
    pf->state_est_deg[AXIS_ROLL] = pf->state_est_rad[AXIS_ROLL] * RAD2DEG;
}

// ===================== 陀螺仪标定：比例+非正交性校正 =====================
/**
 * @brief  应用陀螺仪的比例系数和非正交性校正矩阵（结合SCData零偏）
 * @param  calib: 标定参数结构体（包含3x3矩阵K）
 * @param  raw_gx, raw_gy, raw_gz: 原始陀螺仪数据（deg/s）
 * @param  out_gx, out_gy, out_gz: 校正后的角速度（deg/s）
 * @note   该函数应在零偏补偿之后调用，或直接用于原始数据
 */
void Gyro_Apply_Scale_Nonortho(Gyro_Calib_t *calib,
                                 float raw_gx, float raw_gy, float raw_gz,
                                 float *out_gx, float *out_gy, float *out_gz)
{
    // 3x3矩阵乘法：校正后 = K * 原始向量
    *out_gx = calib->K[0][0] * raw_gx + calib->K[0][1] * raw_gy + calib->K[0][2] * raw_gz;
    *out_gy = calib->K[1][0] * raw_gx + calib->K[1][1] * raw_gy + calib->K[1][2] * raw_gz;
    *out_gz = calib->K[2][0] * raw_gx + calib->K[2][1] * raw_gy + calib->K[2][2] * raw_gz;
}

// ===================== 初始化标定矩阵（示例数据，需根据实际传感器调整） =====================
void Gyro_Calib_Init(Gyro_Calib_t *calib)
{
    // 此处矩阵为示例值（包含比例因子和非正交校正），实际需通过标定实验获得
    // 对角线为比例因子，非对角线为轴间耦合系数
    calib->K[0][0] = 3.679275f;  // X轴比例因子
    calib->K[0][1] = 0.009030f;  // X轴对Y轴耦合
    calib->K[0][2] = 0.016253f;  // X轴对Z轴耦合
    calib->K[1][0] = -0.012518f; // Y轴对X轴耦合
    calib->K[1][1] = 4.269273f;  // Y轴比例因子
    calib->K[1][2] = 0.010439f;  // Y轴对Z轴耦合
    calib->K[2][0] = -0.016767f; // Z轴对X轴耦合
    calib->K[2][1] = 0.010197f;  // Z轴对Y轴耦合
    calib->K[2][2] = 4.338457f;  // Z轴比例因子
}