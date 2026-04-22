#include "main.h"
#include "lib_akf.h"
#include "math.h"
#include "stdint.h"
#include "string.h"

/* ======================================================================
 * Welford 在线方差计算模块
 * 用于实时估算一组数据的样本方差，适用于嵌入式系统中的动态噪声评估。
 * 通常配合 BMI088 等 IMU 使用，在 main.c 中调用以监控陀螺仪稳定性。
 * ====================================================================== */

/**
 * @brief  初始化方差计算器
 * @param  vc:           指向 VarianceCalc_t 结构体的指针
 * @param  target_count: 需要累积多少个样本后才认为方差计算完成（即 ready = 1）
 */
void VarianceCalc_Init(VarianceCalc_t *vc, uint16_t target_count)
{
    memset(vc, 0, sizeof(VarianceCalc_t));
    vc->target_count = target_count;
    vc->ready        = 0;          // 标志位：0 表示尚未累积足够样本，1 表示可读取结果
    vc->mean         = 0.0;        // 当前均值
    vc->M2           = 0.0;        // 二阶中心矩（用于计算方差）
    vc->count        = 0;          // 已接收的样本数量
}

/**
 * @brief  更新方差计算器，传入一个新的采样值
 *         此函数使用 Welford 算法在线更新均值和方差，数值稳定性好。
 * @param  vc:        指向 VarianceCalc_t 的指针
 * @param  new_value: 新的采样值（例如陀螺仪角速度）
 */
void VarianceCalc_UpdateIRQ(VarianceCalc_t *vc, float new_value)
{
    double delta = (double)new_value - vc->mean;

    vc->count++;
    vc->mean += delta / vc->count;               // 更新均值

    double delta2 = (double)new_value - vc->mean;
    vc->M2 += delta * delta2;                    // 更新 M2（二阶中心矩）

    // 当累积样本数达到目标值时，计算样本方差
    if (vc->count >= vc->target_count)
    {
        if (vc->count > 1)
            vc->variance = (float)(vc->M2 / (vc->count - 1.0));  // 无偏样本方差
        else
            vc->variance = 0.0f;                                 // 不足两个样本，方差为0

        // 防止浮点误差导致负方差
        if (vc->variance < 0.0f)
            vc->variance = 0.0f;

        vc->ready = 1;  // 标记结果已就绪
    }
}

/**
 * @brief  获取当前方差计算结果，并重置内部状态（以便下一轮计算）
 * @param  vc:            指向 VarianceCalc_t 的指针
 * @param  out_variance:  输出参数，用于返回计算得到的方差
 * @retval 1 = 成功获取结果并重置；0 = 尚未准备好（样本不足）
 */
uint8_t VarianceCalc_GetResult(VarianceCalc_t *vc, float *out_variance)
{
    if (vc->ready)
    {
        *out_variance = vc->variance;

        // 重置状态，准备下一次方差计算
        vc->mean  = 0.0;
        vc->M2    = 0.0;
        vc->count = 0;
        vc->ready = 0;
        return 1;
    }
    return 0;
}

/* ======================================================================
 * 自适应卡尔曼滤波器（Adaptive Kalman Filter, AKF）
 * 用于对陀螺仪数据进行滤波，其过程噪声协方差 Q 会根据创新序列（Innovation）
 * 的统计特性自动调整，从而在动态与静态场景下均能保持良好性能。
 * ====================================================================== */

/**
 * @brief  初始化陀螺仪自适应卡尔曼滤波器
 * @param  akf:  指向 Gyro_AKF_HandleTypeDef 结构体的指针
 * @param  x0:   初始状态估计值（通常设为第一个测量值或0）
 * @param  P0:   初始估计误差协方差（不确定性），建议设为 1.0f
 * @param  Q0:   初始过程噪声协方差，反映模型不确定性，典型值如 1e-5f
 * @param  R0:   测量噪声协方差，反映传感器噪声水平，需根据实际标定
 */
void Gyro_AKF_Init(Gyro_AKF_HandleTypeDef *akf, float x0, float P0, float Q0, float R0)
{
    memset(akf, 0, sizeof(Gyro_AKF_HandleTypeDef));

    akf->x_hat      = x0;      // 初始状态估计
    akf->P          = P0;      // 初始估计误差协方差
    akf->Q          = Q0;      // 初始过程噪声
    *(float *)&akf->R = R0;    // 绕过 const 限制写入 R（若 R 被声明为 const）

    akf->window_idx  = 0;      // 创新窗口索引
    akf->window_full = 0;      // 标记窗口是否已填满
}

/**
 * @brief  执行一次 AKF 滤波更新
 *         该函数实现标准卡尔曼滤波流程，并通过滑动窗口估计创新方差，
 *         动态调整过程噪声 Q，使滤波器能自适应不同运动状态。
 * @param  z:  当前时刻的陀螺仪测量值
 * @retval     滤波后的状态估计值（即去噪后的角速度）
 */
float Gyro_AKF_Update(Gyro_AKF_HandleTypeDef *akf, float z)
{
    /* 步骤1：状态预测（先验估计） X_{k|k-1} = X_{k-1} */
    akf->x_pre = akf->x_hat;

    /* 步骤3：预测误差协方差更新 P_{k|k-1} = P_{k-1} + Q_k */
    akf->P_pre = akf->P + akf->Q;

    /* 步骤4：计算卡尔曼增益 K = P_{k|k-1} / (P_{k|k-1} + R) */
    float denom = akf->P_pre + akf->R;
    akf->K = (fabsf(denom) < 1e-10f) ? 0.0f : akf->P_pre / denom;  // 避免除零

    /* 计算创新（Innovation）：实际测量与预测值之差 IV = z - x_pre */
    akf->IV = z - akf->x_pre;

    /* 将当前创新存入滑动窗口，用于后续 Q 自适应调整 */
    akf->iv_window[akf->window_idx] = akf->IV;
    akf->window_idx++;
    if (akf->window_idx >= WINDOW_SIZE)
    {
        akf->window_idx  = 0;
        akf->window_full = 1;  // 窗口已满，可开始自适应
    }

    /* 步骤6：自适应调整过程噪声 Q
     * 若窗口已满，则计算创新序列的方差 C，
     * 并令 Q = K? * C，使滤波器在高动态时增大 Q（信任模型更少，信任测量更多）
     */
    if (akf->window_full)
    {
        akf->C = 0.0f;
        for (uint8_t i = 0; i < WINDOW_SIZE; i++)
            akf->C += akf->iv_window[i] * akf->iv_window[i];
        akf->C /= WINDOW_SIZE;  // 创新方差估计

        akf->Q = akf->K * akf->K * akf->C;
        if (akf->Q < 1e-8f)     // 设置 Q 下限，防止数值不稳定
            akf->Q = 1e-8f;
    }

    /* 步骤2：状态更新（后验估计） X_k = X_{k|k-1} + K * IV */
    akf->x_hat = akf->x_pre + akf->K * akf->IV;

    /* 步骤5：更新估计误差协方差 P_k = (1 - K) * P_{k|k-1} */
    akf->P = (1.0f - akf->K) * akf->P_pre;
    if (akf->P < 0.0f)          // 防止协方差变为负值
        akf->P = 1e-8f;

    return akf->x_hat;
}