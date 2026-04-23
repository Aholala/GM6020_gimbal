#include "main.h"
#include "lib_AKF.h"
#include "math.h"
#include "stdint.h"
#include "string.h"

/* ======================================================================
 * Welford 在线方差（中断版）
 * 供需要在定时器中断里持续采集方差的场合使用。
 * BMI088 工程目前在 main.c 校准段内联使用，此处保留以备扩展。
 * ====================================================================== */
void VarianceCalc_Init(VarianceCalc_t *vc, uint16_t target_count)
{
    memset(vc, 0, sizeof(VarianceCalc_t));
    vc->target_count = target_count;
    vc->ready        = 0;
    vc->mean         = 0.0;
    vc->M2           = 0.0;
    vc->count        = 0;
}

/**
 * @brief  中断中更新 1 个样本（快进快出，无阻塞）
 * @param  vc:        句柄指针
 * @param  new_value: 当前采样值
 */
void VarianceCalc_UpdateIRQ(VarianceCalc_t *vc, float new_value)
{
    double delta = (double)new_value - vc->mean;

    vc->count++;
    vc->mean += delta / vc->count;

    double delta2 = (double)new_value - vc->mean;
    vc->M2 += delta * delta2;

    if (vc->count >= vc->target_count)
    {
        if (vc->count > 1)
            vc->variance = (float)(vc->M2 / (vc->count - 1.0));
        else
            vc->variance = 0.0f;

        if (vc->variance < 0.0f)
            vc->variance = 0.0f;

        vc->ready = 1;
    }
}

/**
 * @brief  主循环读取方差结果（读取后自动重置）
 * @retval 1=有新结果，0=未就绪
 */
uint8_t VarianceCalc_GetResult(VarianceCalc_t *vc, float *out_variance)
{
    if (vc->ready)
    {
        *out_variance = vc->variance;
        vc->mean  = 0.0;
        vc->M2    = 0.0;
        vc->count = 0;
        vc->ready = 0;
        return 1;
    }
    return 0;
}

/* ======================================================================
 * 陀螺仪自适应卡尔曼滤波（AKF）
 * ====================================================================== */

/**
 * @brief  AKF 初始化
 * @param  x0: 初始状态估计值（静止时填零偏均值）
 * @param  P0: 初始协方差（推荐 1.0f）
 * @param  Q0: 初始系统噪声方差（推荐 1e-5f，后续自适应更新）
 * @param  R0: 量测噪声方差（静止标定方差，运行全程固定不变）
 */
void Gyro_AKF_Init(Gyro_AKF_HandleTypeDef *akf, float x0, float P0, float Q0, float R0)
{
    memset(akf, 0, sizeof(Gyro_AKF_HandleTypeDef));

    akf->x_hat      = x0;
    akf->P          = P0;
    akf->Q          = Q0;
    *(float *)&akf->R = R0;   /* const 成员只能这样写入 */

    akf->window_idx  = 0;
    akf->window_full = 0;
}

/**
 * @brief  AKF 单步更新
 * @param  z: 当前原始角速度量测值
 * @retval 滤波后的角速度最优估计值
 */
float Gyro_AKF_Update(Gyro_AKF_HandleTypeDef *akf, float z)
{
    /* 步骤1：状态一步预测  X_{k|k-1} = X_{k-1} */
    akf->x_pre = akf->x_hat;

    /* 步骤3：协方差一步预测  P_{k|k-1} = P_{k-1} + Q_k */
    akf->P_pre = akf->P + akf->Q;

    /* 步骤4：卡尔曼增益  K = P_{k|k-1} / (P_{k|k-1} + R) */
    float denom = akf->P_pre + akf->R;
    akf->K = (fabsf(denom) < 1e-10f) ? 0.0f : akf->P_pre / denom;

    /* 新息 */
    akf->IV = z - akf->x_pre;

    /* 更新新息滑动窗口 */
    akf->iv_window[akf->window_idx] = akf->IV;
    akf->window_idx++;
    if (akf->window_idx >= WINDOW_SIZE)
    {
        akf->window_idx  = 0;
        akf->window_full = 1;
    }

    /* 步骤6：自适应更新 Q（窗口满后启动） */
    if (akf->window_full)
    {
        akf->C = 0.0f;
        for (uint8_t i = 0; i < WINDOW_SIZE; i++)
            akf->C += akf->iv_window[i] * akf->iv_window[i];
        akf->C /= WINDOW_SIZE;

        akf->Q = akf->K * akf->K * akf->C;
        if (akf->Q < 1e-8f)
            akf->Q = 1e-8f;
    }

    /* 步骤2：状态更新  X_k = X_{k|k-1} + K * IV */
    akf->x_hat = akf->x_pre + akf->K * akf->IV;

    /* 步骤5：协方差更新  P_k = (1 - K) * P_{k|k-1} */
    akf->P = (1.0f - akf->K) * akf->P_pre;
    if (akf->P < 0.0f)
        akf->P = 1e-8f;

    return akf->x_hat;
}