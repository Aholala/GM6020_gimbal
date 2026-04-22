#ifndef __TOOLS_H
#define __TOOLS_H

#include <stdint.h>

#define WINDOW_SIZE  10

/************************** 陀螺仪AKF结构体定义 **************************/
typedef struct {
    // 核心状态量
    float x_hat;          // 滤波后的角速度最优估计值（最终输出）
    float x_pre;          // 状态一步预测值 对应论文式(3.5)
    float P;              // 状态估计误差协方差 对应论文式(3.9)输出
    float P_pre;          // 协方差一步预测值 对应论文式(3.7)
    float K;              // 卡尔曼滤波增益 对应论文式(3.8)

    // 噪声参数
    float Q;              // 系统噪声方差 自适应实时更新 对应论文式(3.10)
    const float R;        // 量测噪声方差 固定值（初始化静止标定，运行不变）

    // 新息相关
    float IV;             // 新息(量测残差) 对应论文式(Z_j - X_j)
    float C;              // 新息的方差 对应论文式(3.11)

    // 新息滑动窗口(循环缓冲区，论文式3.11用)
    float iv_window[WINDOW_SIZE];  // 窗口内的新息序列
    uint8_t window_idx;            // 窗口当前索引
    uint8_t window_full;           // 窗口是否填满(填满后才启动Q自适应)
} Gyro_AKF_HandleTypeDef;

// 方差计算句柄：单轴独立使用
typedef struct {
    // 配置参数
    uint16_t target_count;  // 目标样本数

    // Welford算法状态变量（必须用double，保证精度）
    double mean;             // 当前均值
    double M2;               // 离均差平方和
    uint16_t count;          // 当前已采集样本数

    // 结果变量
    float variance;          // 最终无偏方差
    volatile uint8_t ready;  // 完成标志（1=结果就绪）
} VarianceCalc_t;

/* AKF 核心接口 */
void  Gyro_AKF_Init(Gyro_AKF_HandleTypeDef *akf, float x0, float P0, float Q0, float R0);
float Gyro_AKF_Update(Gyro_AKF_HandleTypeDef *akf, float z);

/* Welford 在线方差（中断版，供需要时使用） */
void    VarianceCalc_Init(VarianceCalc_t *vc, uint16_t sample_count);
void    VarianceCalc_UpdateIRQ(VarianceCalc_t *vc, float filtered_value);
uint8_t VarianceCalc_GetResult(VarianceCalc_t *vc, float *out_variance);

#endif