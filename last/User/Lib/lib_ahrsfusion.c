#include "math.h"
#include "lib_akf.h"
#include "lib_ahrsfusion.h"
#include <stdlib.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*************************** 角度单位转换常量 ******************************/
#define DEG2RAD  0.017453292519943295f  // 度转弧度（π/180）
#define RAD2DEG  57.29577951308232f     // 弧度转度（180/π）
/**************************************************************************/

// 静态检测阈值：用于判断 IMU 是否处于静止状态
#define STABILITY_THRESHOLD  0.001f   // 角速度变化小于 0.001 rad/s（约 0.057°/s）
#define STABILITY_COUNT      100     // 连续 100 次（假设 1ms 采样，即 100ms）满足条件才认为静止

/************************** Mahony AHRS 参数配置 **************************/
// 比例增益 KP 和积分增益 KI 根据运动状态动态切换
#define KP_STATIC   4.0f    // 静止时：高比例增益，快速对齐重力方向
#define KP_DYNAMIC  0.01f   // 动态时：低比例增益，减少加速度扰动影响
#define KI_STATIC   0.002f  // 静止时：启用积分项补偿陀螺仪漂移
#define KI_DYNAMIC  0.0f    // 动态时：禁用积分，防止加速度误差累积
#define INTEGRAL_MAX_LIMIT  0.05f  // 积分项上限（rad/s）
#define INTEGRAL_MIN_LIMIT -0.05f  // 积分项下限（rad/s）

// 零速更新（ZUPT）相关参数：用于抑制 Yaw 轴漂移
#define GYRO_STATIC_THRESHOLD  2.0f  // 判定静止的角速度阈值（deg/s）
#define GYRO_HYSTERESIS        0.5f  // 滞环宽度，防止状态抖动

/************************** 梯度补偿使能开关 ******************************/
#define ENABLE_GRADIENT_COMPENSATE  1  // 1=启用偏置梯度补偿；0=禁用
#define COMPENSATE_INTERVAL         10  // 每 10ms（100Hz）尝试一次补偿

/************************** 补偿安全限制 ******************************/
#define COMPENSATE_STEP_SAFE       0.0001f // 单次最大补偿步长（rad/s）
#define BIAS_MAX_LIMIT_SAFE        1.0f     // 偏置上限（rad/s）
#define BIAS_MIN_LIMIT_SAFE       -1.0f     // 偏置下限（rad/s）
/**************************************************************************/

/* 注意：IMU_Calibrate_Accel_Offset 函数未在此文件定义，通常用于加速度计零偏校准 */

/**
 * @brief  初始化静态补偿数据结构
 * @param  SCData: 指向静态补偿结构体的指针
 * @note   所有偏置初始为 0，校准状态标记为“未校准”
 */
void SC_Data_Init(SC_Data* SCData)
{
    SCData->gyro_x_SC = 0.0f;
    SCData->gyro_y_SC = 0.0f;
    SCData->gyro_z_SC = 0.0f;
    SCData->gyro_x_bias = 0.0f;
    SCData->gyro_y_bias = 0.0f;
    SCData->gyro_z_bias = 0.0f;
    SCData->is_calibrated = 0;  // 标记为未校准
}

/**
 * @brief  执行陀螺仪静态偏置补偿
 *         先通过 AKF 滤波，再检测是否静止，若静止则估计偏置并补偿
 * @param  MPU_Data: 原始 IMU 数据（含 gx, gy, gz，单位 deg/s）
 * @param  gyro_AKF: 存储 AKF 滤波后结果的结构体
 * @param  SCData: 静态补偿数据（含偏置和补偿后输出）
 * @param  AKF_X/Y/Z: 三个轴的自适应卡尔曼滤波器句柄
 */
void static_compensation(MPU_Data* MPU_Data, AKF_Data* gyro_AKF, SC_Data* SCData, Gyro_AKF_HandleTypeDef* AKF_X, Gyro_AKF_HandleTypeDef* AKF_Y, Gyro_AKF_HandleTypeDef* AKF_Z)
{
    // 保存上一时刻的滤波后角速度，用于静止检测
    static float prev_gx = 0.0f, prev_gy = 0.0f, prev_gz = 0.0f;
    static uint16_t stable_cnt = 0;

    // 对三轴原始陀螺仪数据进行 AKF 滤波
    gyro_AKF->gyro_x_hat = Gyro_AKF_Update(AKF_X, MPU_Data->gx);
    gyro_AKF->gyro_y_hat = Gyro_AKF_Update(AKF_Y, MPU_Data->gy);
    gyro_AKF->gyro_z_hat = Gyro_AKF_Update(AKF_Z, MPU_Data->gz);

    // 若尚未完成校准，则尝试检测静止并估计偏置
    if (SCData->is_calibrated == 0)
    {
        // 计算当前与上一时刻角速度的变化量（绝对值）
        float dx = fabs(gyro_AKF->gyro_x_hat - prev_gx);
        float dy = fabs(gyro_AKF->gyro_y_hat - prev_gy);
        float dz = fabs(gyro_AKF->gyro_z_hat - prev_gz);

        // 更新历史值
        prev_gx = gyro_AKF->gyro_x_hat;
        prev_gy = gyro_AKF->gyro_y_hat;
        prev_gz = gyro_AKF->gyro_z_hat;

        // 若三轴变化均小于阈值，则计数器递增
        if (dx < STABILITY_THRESHOLD && dy < STABILITY_THRESHOLD && dz < STABILITY_THRESHOLD)
        {
            stable_cnt++;
            // 连续稳定足够长时间，认为处于静止状态
            if (stable_cnt >= STABILITY_COUNT)
            {
                // 将当前滤波后的角速度作为偏置（理想静止时应为 0）
                SCData->gyro_x_bias = gyro_AKF->gyro_x_hat;
                SCData->gyro_y_bias = gyro_AKF->gyro_y_hat;
                SCData->gyro_z_bias = gyro_AKF->gyro_z_hat;
                SCData->is_calibrated = 1;  // 标记校准完成
                stable_cnt = 0;             // 重置计数器
            }
        }
        else
        {
            stable_cnt = 0;  // 一旦不稳定，立即清零计数
        }

        // 输出补偿后的角速度（即使未校准也减去当前偏置估计，初始为 0）
        SCData->gyro_x_SC = gyro_AKF->gyro_x_hat - SCData->gyro_x_bias;
        SCData->gyro_y_SC = gyro_AKF->gyro_y_hat - SCData->gyro_y_bias;
        SCData->gyro_z_SC = gyro_AKF->gyro_z_hat - SCData->gyro_z_bias;
    }
    else
    {
        // 已校准：直接使用固定偏置进行补偿
        SCData->gyro_x_SC = gyro_AKF->gyro_x_hat - SCData->gyro_x_bias;
        SCData->gyro_y_SC = gyro_AKF->gyro_y_hat - SCData->gyro_y_bias;
        SCData->gyro_z_SC = gyro_AKF->gyro_z_hat - SCData->gyro_z_bias;
    }
}

/**************************************************************************
 * @brief  Mahony AHRS 算法初始化
 *         将四元数初始化为单位四元数（表示无旋转），清除积分项
 * @param  ahrs: 指向 Mahony_AHRS_t 结构体的指针
 **************************************************************************/
void Mahony_Init(Mahony_AHRS_t *ahrs)
{
    // 1. 四元数初始化：q = [1, 0, 0, 0] 表示世界坐标系与机体坐标系对齐
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    // 2. 清除积分项（用于补偿陀螺仪漂移）
    ahrs->exInt = 0.0f;
    ahrs->eyInt = 0.0f;
    ahrs->ezInt = 0.0f;

    // 3. 平滑变量（本实现中未使用，保留扩展接口）
    ahrs->delta_acc_smooth = 0.0f;
    ahrs->gyro_energy_smooth = 0.0f;
    ahrs->alpha_smooth = 0.0f;

    // 4. Yaw 轴零偏与锁定标志（用于 ZUPT）
    ahrs->yaw_bias = 0.0f;          // Yaw 轴零偏估计
    ahrs->yaw_static_ref = 0.0f;    // 静止时的 Yaw 参考（本实现未用）
    ahrs->yaw_locked = 0;           // Yaw 是否被锁定（本实现未用）
}

/**************************************************************************
 * @brief  判断 IMU 是否真正处于静止状态（用于 ZUPT）
 *         使用滞环比较避免状态抖动
 * @param  gx_sc, gy_sc, gz_sc: 静态补偿后的角速度（单位：deg/s）
 * @retval 1 = 静止；0 = 动态
 **************************************************************************/
uint8_t Is_Really_Static(float gx_sc, float gy_sc, float gz_sc)
{
    static uint8_t is_static_last = 1;  // 上一状态默认为静止

    // 1. 计算角速度模长（L2 范数）
    float gyro_norm = sqrt(gx_sc*gx_sc + gy_sc*gy_sc + gz_sc*gz_sc);
    uint8_t gyro_static = 0;

    // 2. 滞环判断：根据上一状态调整阈值
    if (is_static_last) {
        // 当前为静止 → 需要更宽松的退出条件（+ 滞环）
        if (gyro_norm < (GYRO_STATIC_THRESHOLD + GYRO_HYSTERESIS)) {
            gyro_static = 1;
        }
    } else {
        // 当前为动态 → 需要更严格的进入条件（- 滞环）
        if (gyro_norm < (GYRO_STATIC_THRESHOLD - GYRO_HYSTERESIS)) {
            gyro_static = 1;
        }
    }

    // 3. 更新并返回当前静止状态
    uint8_t is_static = gyro_static;
    is_static_last = is_static;
    return is_static;
}

/**************************************************************************
 * @brief  使用加速度计初始对齐（仅 Roll/Pitch），Yaw 初始化为 0
 *         适用于启动时设备大致水平放置的场景
 * @param  ahrs: AHRS 结构体指针
 * @param  ax, ay, az: 加速度计原始数据（单位：g，需归一化）
 **************************************************************************/
void Mahony_Init_With_Calibration(Mahony_AHRS_t *ahrs, float ax, float ay, float az)
{
    // 1. 归一化加速度向量（消除量程影响）
    float norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm > 0.0001f) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    // 2. 由重力方向反解初始 Roll 和 Pitch（假设无横向加速度）
    float init_roll = atan2(ay, az);        // Roll = arctan(ay/az)
    float init_pitch = -asin(ax);           // Pitch = -arcsin(ax)

    // 3. 构造对应四元数（Yaw = 0）
    float cr = cos(init_roll * 0.5f);
    float sr = sin(init_roll * 0.5f);
    float cp = cos(init_pitch * 0.5f);
    float sp = sin(init_pitch * 0.5f);

    // q = [cos(φ/2)cos(θ/2), sin(φ/2)cos(θ/2), cos(φ/2)sin(θ/2), -sin(φ/2)sin(θ/2)]
    ahrs->q0 = cr * cp;
    ahrs->q1 = sr * cp;
    ahrs->q2 = cr * sp;
    ahrs->q3 = -sr * sp;

    // 4. 清除积分项
    ahrs->exInt = 0.0f;
    ahrs->eyInt = 0.0f;
    ahrs->ezInt = 0.0f;
}

/**************************************************************************
 * @brief  Mahony AHRS 主更新函数（含 ZUPT 和 RK2 积分）
 *         动态切换 KP/KI，并在静止时对 Yaw 轴进行零偏估计
 * @param  ahrs: AHRS 结构体
 * @param  ax, ay, az: 加速度计数据（单位：g）
 * @param  wx_deg, wy_deg, wz_deg: 陀螺仪数据（单位：deg/s）
 * @param  is_static: 是否静止（由 Is_Really_Static 提供）
 * @param  dt: 采样周期（秒）
 **************************************************************************/
void Mahony_Update(Mahony_AHRS_t *ahrs, float ax, float ay, float az, float wx_deg, float wy_deg, float wz_deg, uint8_t is_static, float dt)
{
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // 1. 根据运动状态选择 AHRS 增益参数
    float Kp = is_static ? KP_STATIC : KP_DYNAMIC;
    float Ki = is_static ? KI_STATIC : KI_DYNAMIC;

    // 2. 角速度单位转换：deg/s → rad/s
    float wx = wx_deg * DEG2RAD;
    float wy = wy_deg * DEG2RAD;
    float wz = wz_deg * DEG2RAD;

    // ========== ZUPT：静止时估计并补偿 Yaw 轴零偏 ==========
    if (is_static)
    {
        const float KP_YAW_SLOW = 0.015f;  // 小误差时慢速修正
        const float KP_YAW_FAST = 0.02f;   // 大误差时快速修正

        float wz_corrected = wz - ahrs->yaw_bias;  // 补偿当前估计的零偏

        // 自适应增益：误差大则快修，误差小则慢修
        float kp_yaw = (fabsf(wz_corrected) > 0.005f) ? KP_YAW_FAST : KP_YAW_SLOW;

        // 积分更新 Yaw 零偏（本质是低通滤波）
        ahrs->yaw_bias += kp_yaw * wz_corrected * dt;

        // 限制零偏范围（±0.1 rad/s ≈ ±5.7°/s）
        const float YAW_BIAS_MAX = 0.1f;
        if (ahrs->yaw_bias > YAW_BIAS_MAX) ahrs->yaw_bias = YAW_BIAS_MAX;
        if (ahrs->yaw_bias < -YAW_BIAS_MAX) ahrs->yaw_bias = -YAW_BIAS_MAX;
    }
    // 无论是否静止，都从角速度中减去当前 Yaw 零偏估计
    wz -= ahrs->yaw_bias;
    // ========== ZUPT 结束 ==========

    // 3. 归一化加速度向量（作为重力参考）
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm <= 0.0001f) return;  // 防止除零
    ax /= norm; ay /= norm; az /= norm;

    // 4. 由当前四元数计算重力在机体坐标系下的预测值
    halfvx = ahrs->q1 * ahrs->q3 - ahrs->q0 * ahrs->q2;
    halfvy = ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3;
    halfvz = ahrs->q0 * ahrs->q0 - 0.5f + ahrs->q3 * ahrs->q3;

    // 5. 计算观测误差（实际重力 vs 预测重力）
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // 6. 积分项更新（仅当 Ki > 0 时启用）
    if (Ki > 0.0f) {
        ahrs->exInt += Ki * halfex * dt;
        ahrs->eyInt += Ki * halfey * dt;
        ahrs->ezInt += Ki * halfez * dt;

        // 限制积分项防 wind-up
        ahrs->exInt = fmaxf(INTEGRAL_MIN_LIMIT, fminf(INTEGRAL_MAX_LIMIT, ahrs->exInt));
        ahrs->eyInt = fmaxf(INTEGRAL_MIN_LIMIT, fminf(INTEGRAL_MAX_LIMIT, ahrs->eyInt));
        ahrs->ezInt = fmaxf(INTEGRAL_MIN_LIMIT, fminf(INTEGRAL_MAX_LIMIT, ahrs->ezInt));

        // 将积分补偿加到角速度上
        wx += ahrs->exInt;
        wy += ahrs->eyInt;
        wz += ahrs->ezInt;
    } else {
        // 动态时清零积分项
        ahrs->exInt = 0.0f;
        ahrs->eyInt = 0.0f;
        ahrs->ezInt = 0.0f;
    }

    // 7. 比例项补偿
    wx += Kp * halfex;
    wy += Kp * halfey;
    wz += Kp * halfez;

    // ========== 使用 RK2（二阶龙格-库塔）进行四元数积分 ==========
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float half_wx = 0.5f * wx;
    float half_wy = 0.5f * wy;
    float half_wz = 0.5f * wz;

    // k1: 使用当前状态计算导数
    float k1_q0 = -q1 * half_wx - q2 * half_wy - q3 * half_wz;
    float k1_q1 =  q0 * half_wx + q2 * half_wz - q3 * half_wy;
    float k1_q2 =  q0 * half_wy - q1 * half_wz + q3 * half_wx;
    float k1_q3 =  q0 * half_wz + q1 * half_wy - q2 * half_wx;

    // 中间状态（半步）
    float half_dt = 0.5f * dt;
    float q_mid0 = q0 + k1_q0 * half_dt;
    float q_mid1 = q1 + k1_q1 * half_dt;
    float q_mid2 = q2 + k1_q2 * half_dt;
    float q_mid3 = q3 + k1_q3 * half_dt;

    // 归一化中间四元数
    norm = sqrtf(q_mid0*q_mid0 + q_mid1*q_mid1 + q_mid2*q_mid2 + q_mid3*q_mid3);
    if (norm > 0.0001f) {
        q_mid0 /= norm; q_mid1 /= norm; q_mid2 /= norm; q_mid3 /= norm;
    } else {
        return;
    }

    // k2: 使用中间状态计算导数
    float k2_q0 = -q_mid1 * half_wx - q_mid2 * half_wy - q_mid3 * half_wz;
    float k2_q1 =  q_mid0 * half_wx + q_mid2 * half_wz - q_mid3 * half_wy;
    float k2_q2 =  q_mid0 * half_wy - q_mid1 * half_wz + q_mid3 * half_wx;
    float k2_q3 =  q_mid0 * half_wz + q_mid1 * half_wy - q_mid2 * half_wx;

    // 最终更新（使用 k2）
    ahrs->q0 = q0 + k2_q0 * dt;
    ahrs->q1 = q1 + k2_q1 * dt;
    ahrs->q2 = q2 + k2_q2 * dt;
    ahrs->q3 = q3 + k2_q3 * dt;

    // 最终归一化
    norm = sqrtf(ahrs->q0*ahrs->q0 + ahrs->q1*ahrs->q1 + ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3);
    if (norm > 0.0001f) {
        ahrs->q0 /= norm; ahrs->q1 /= norm; ahrs->q2 /= norm; ahrs->q3 /= norm;
    }
    // ========== RK2 积分结束 ==========
}

/**************************************************************************
 * @brief  从四元数解算欧拉角（弧度）
 *         采用 ZYX 旋转顺序（Yaw-Pitch-Roll）
 * @param  ahrs: AHRS 结构体
 * @param  roll_rad, pitch_rad, yaw_rad: 输出指针
 **************************************************************************/
void Mahony_Get_Euler_Rad(Mahony_AHRS_t *ahrs, float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    *roll_rad = atan2(2.0f * (ahrs->q0*ahrs->q1 + ahrs->q2*ahrs->q3), 1.0f - 2.0f * (ahrs->q1*ahrs->q1 + ahrs->q2*ahrs->q2));
    *pitch_rad = asin(2.0f * (ahrs->q0*ahrs->q2 - ahrs->q3*ahrs->q1));
    *yaw_rad = atan2(2.0f * (ahrs->q0*ahrs->q3 + ahrs->q1*ahrs->q2), 1.0f - 2.0f * (ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3));
}

/**************************************************************************
 * @brief  从四元数解算欧拉角（角度）
 **************************************************************************/
void Mahony_Get_Euler_Deg(Mahony_AHRS_t *ahrs, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll_rad, pitch_rad, yaw_rad;
    Mahony_Get_Euler_Rad(ahrs, &roll_rad, &pitch_rad, &yaw_rad);
    *roll_deg = roll_rad * RAD2DEG;
    *pitch_deg = pitch_rad * RAD2DEG;
    *yaw_deg = yaw_rad * RAD2DEG;
}

// ===================== 辅助函数：角度归一化到 [-π, π) =====================
static float pf_wrap_angle_rad(float angle_rad) {
    while (angle_rad >= M_PI) angle_rad -= 2.0f * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0f * M_PI;
    return angle_rad;
}

// ===================== 辅助函数：角度归一化到 [-180°, 180°) =====================
static float pf_wrap_angle_deg(float angle_deg) {
    while (angle_deg >= 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

// ===================== 辅助函数：度转弧度（先归一化） =====================
static float pf_deg_to_rad(float angle_deg) {
    return pf_wrap_angle_deg(angle_deg) * DEG2RAD;
}

// ===================== 辅助函数：两角度之差（弧度） =====================
static float pf_angle_diff_rad(float angle1_rad, float angle2_rad) {
    return pf_wrap_angle_rad(angle1_rad - angle2_rad);
}

// ===================== 辅助函数：生成高斯白噪声（Box-Muller 变换） =====================
static float pf_gaussian_rand(float std) {
    static int has_spare = 0;
    static float spare = 0.0f;

    if (has_spare) {
        has_spare = 0;
        return spare * std;
    }

    float u1, u2, s;
    do {
        u1 = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        u2 = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        s = u1*u1 + u2*u2;
    } while (s >= 1.0f || s == 0.0f);

    s = sqrtf(-2.0f * logf(s) / s);
    spare = u2 * s;
    has_spare = 1;
    return u1 * s * std;
}

// ===================== 粒子滤波器初始化 =====================
/**
 * @brief  初始化粒子滤波器（用于优化 Pitch/Roll）
 *         初始状态由外部提供（通常来自 Mahony 初始对齐）
 * @param  pf: 粒子滤波器结构体
 * @param  init_pitch_deg, init_roll_deg: 初始角度（度）
 */
void pf_init(ParticleFilter_t *pf, float init_pitch_deg, float init_roll_deg) {
    float init_pitch_rad = pf_deg_to_rad(init_pitch_deg);
    float init_roll_rad = pf_deg_to_rad(init_roll_deg);

    // 设置过程噪声和观测噪声标准差（可调参数）
    pf->process_noise_std[AXIS_PITCH] = 0.002f;  // ~0.11°
    pf->measure_noise_std[AXIS_PITCH] = 0.006f;  // ~0.34°
    pf->process_noise_std[AXIS_ROLL]  = 0.002f;
    pf->measure_noise_std[AXIS_ROLL]  = 0.006f;

    // 初始化粒子：围绕初始值加高斯噪声
    float init_std = 0.035f; // ~2°
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        pf->particles[i][AXIS_PITCH] = pf_wrap_angle_rad(init_pitch_rad + pf_gaussian_rand(init_std));
        pf->particles[i][AXIS_ROLL]  = pf_wrap_angle_rad(init_roll_rad + pf_gaussian_rand(init_std));
    }

    // 均匀初始化权重
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        pf->weights[i] = 1.0f / PF_PARTICLE_NUM;
    }

    // 初始化状态估计
    pf->state_est_rad[AXIS_PITCH] = init_pitch_rad;
    pf->state_est_rad[AXIS_ROLL]  = init_roll_rad;
    pf->state_est_deg[AXIS_PITCH] = init_pitch_deg;
    pf->state_est_deg[AXIS_ROLL]  = init_roll_deg;
}

// ===================== 粒子预测步骤 =====================
void pf_predict(ParticleFilter_t *pf) {
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        // 为每个粒子添加过程噪声（模拟系统不确定性）
        pf->particles[i][AXIS_PITCH] = pf_wrap_angle_rad(pf->particles[i][AXIS_PITCH] + pf_gaussian_rand(pf->process_noise_std[AXIS_PITCH]));
        pf->particles[i][AXIS_ROLL]  = pf_wrap_angle_rad(pf->particles[i][AXIS_ROLL]  + pf_gaussian_rand(pf->process_noise_std[AXIS_ROLL]));
    }
}

// ===================== 粒子更新步骤 =====================
/**
 * @brief  根据 Mahony 融合结果更新粒子权重
 * @param  fused_pitch_deg, fused_roll_deg: Mahony 输出的角度（作为观测值）
 */
void pf_update(ParticleFilter_t *pf, float fused_pitch_deg, float fused_roll_deg) {
    float fused_pitch_rad = pf_deg_to_rad(fused_pitch_deg);
    float fused_roll_rad = pf_deg_to_rad(fused_roll_deg);

    float R_pitch = pf->measure_noise_std[AXIS_PITCH] * pf->measure_noise_std[AXIS_PITCH];
    float R_roll  = pf->measure_noise_std[AXIS_ROLL]  * pf->measure_noise_std[AXIS_ROLL];

    float weight_sum = 0.0f;

    // 计算每个粒子的似然（高斯分布）
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        float error_pitch = pf_angle_diff_rad(fused_pitch_rad, pf->particles[i][AXIS_PITCH]);
        float error_roll  = pf_angle_diff_rad(fused_roll_rad,  pf->particles[i][AXIS_ROLL]);

        float likelihood_pitch = expf(-0.5f * error_pitch * error_pitch / R_pitch);
        float likelihood_roll  = expf(-0.5f * error_roll  * error_roll  / R_roll);

        pf->weights[i] *= likelihood_pitch * likelihood_roll;
        weight_sum += pf->weights[i];
    }

    // 归一化权重（防退化）
    if (weight_sum < 1e-10f) {
        // 权重崩溃：重新均匀初始化
        for (int i = 0; i < PF_PARTICLE_NUM; i++) {
            pf->weights[i] = 1.0f / PF_PARTICLE_NUM;
        }
    } else {
        for (int i = 0; i < PF_PARTICLE_NUM; i++) {
            pf->weights[i] /= weight_sum;
        }
    }
}

// ===================== 计算有效粒子数（Neff） =====================
float pf_calculate_neff(ParticleFilter_t *pf) {
    float sum_sq = 0.0f;
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        sum_sq += pf->weights[i] * pf->weights[i];
    }
    return 1.0f / sum_sq;
}

// ===================== 重采样（低方差采样） =====================
void pf_resample(ParticleFilter_t *pf) {
    float new_particles[PF_PARTICLE_NUM][PF_STATE_DIM];
    float step = 1.0f / PF_PARTICLE_NUM;
    float u = ((float)rand() / RAND_MAX) * step;
    float cumulative_sum = pf->weights[0];
    int j = 0;

    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        while (u > cumulative_sum) {
            j++;
            cumulative_sum += pf->weights[j];
        }
        new_particles[i][AXIS_PITCH] = pf_wrap_angle_rad(pf->particles[j][AXIS_PITCH]);
        new_particles[i][AXIS_ROLL]  = pf_wrap_angle_rad(pf->particles[j][AXIS_ROLL]);
        u += step;
    }

    // 替换旧粒子，权重重置为均匀
    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        pf->particles[i][AXIS_PITCH] = new_particles[i][AXIS_PITCH];
        pf->particles[i][AXIS_ROLL]  = new_particles[i][AXIS_ROLL];
        pf->weights[i] = 1.0f / PF_PARTICLE_NUM;
    }
}

// ===================== 状态估计（加权平均） =====================
void pf_estimate(ParticleFilter_t *pf) {
    float sum_sin_pitch = 0.0f, sum_cos_pitch = 0.0f;
    float sum_sin_roll = 0.0f, sum_cos_roll = 0.0f;

    for (int i = 0; i < PF_PARTICLE_NUM; i++) {
        float w = pf->weights[i];
        sum_sin_pitch += w * sinf(pf->particles[i][AXIS_PITCH]);
        sum_cos_pitch += w * cosf(pf->particles[i][AXIS_PITCH]);
        sum_sin_roll  += w * sinf(pf->particles[i][AXIS_ROLL]);
        sum_cos_roll  += w * cosf(pf->particles[i][AXIS_ROLL]);
    }

    // 使用 atan2 避免角度歧义
    pf->state_est_rad[AXIS_PITCH] = atan2f(sum_sin_pitch, sum_cos_pitch);
    pf->state_est_rad[AXIS_ROLL]  = atan2f(sum_sin_roll,  sum_cos_roll);

    pf->state_est_deg[AXIS_PITCH] = pf->state_est_rad[AXIS_PITCH] * RAD2DEG;
    pf->state_est_deg[AXIS_ROLL]  = pf->state_est_rad[AXIS_ROLL]  * RAD2DEG;
}

// ===================== 陀螺仪非正交与尺度校准 =====================
/**
 * @brief  应用 3x3 校准矩阵对原始陀螺仪数据进行补偿
 *         补偿包括：尺度因子、轴间非正交误差、交叉耦合
 * @param  calib: 校准参数结构体（含 3x3 矩阵 K）
 * @param  raw_gx/y/z: 原始角速度（deg/s）
 * @param  out_gx/y/z: 校准后输出（deg/s）
 */
void Gyro_Apply_Scale_Nonortho(Gyro_Calib_t *calib,
                                 float raw_gx, float raw_gy, float raw_gz,
                                 float *out_gx, float *out_gy, float *out_gz)
{
    *out_gx = calib->K[0][0] * raw_gx + calib->K[0][1] * raw_gy + calib->K[0][2] * raw_gz;
    *out_gy = calib->K[1][0] * raw_gx + calib->K[1][1] * raw_gy + calib->K[1][2] * raw_gz;
    *out_gz = calib->K[2][0] * raw_gx + calib->K[2][1] * raw_gy + calib->K[2][2] * raw_gz;
}

// ===================== 陀螺仪校准参数初始化 =====================
/**
 * @brief  初始化预标定的陀螺仪校准矩阵
 *         这些参数通常通过六面法或椭球拟合标定得到
 */
void Gyro_Calib_Init(Gyro_Calib_t *calib)
{
    // 示例参数（实际值需根据具体传感器标定）
    calib->K[0][0] = 3.679275f;  // X 轴灵敏度（LSB/(°/s) 的倒数）
    calib->K[0][1] = 0.009030f;  // X-Y 轴非正交耦合
    calib->K[0][2] = 0.016253f;  // X-Z 轴非正交耦合
    calib->K[1][0] = -0.012518f; // Y-X 轴非正交耦合
    calib->K[1][1] = 4.269273f;  // Y 轴灵敏度
    calib->K[1][2] = 0.010439f;  // Y-Z 轴非正交耦合
    calib->K[2][0] = -0.016767f; // Z-X 轴非正交耦合
    calib->K[2][1] = 0.010197f;  // Z-Y 轴非正交耦合
    calib->K[2][2] = 4.338457f;  // Z 轴灵敏度
}