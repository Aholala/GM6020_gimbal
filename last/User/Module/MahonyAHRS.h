#ifndef MahonyAHRS_h
#define MahonyAHRS_h

/* =====================================================================
 * Mahony AHRS 姿态融合算法接口
 *
 * 调用方式（固定频率版，本文件 sampleFreq = 1000Hz，主循环按实际频率调整）：
 *   float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始化一次
 *   MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);  // 每周期调用
 *
 * 四元数 → 欧拉角（ZYX，单位 rad）：
 *   roll  = atan2(2(w·x + y·z), 1 - 2(x²+y²))
 *   pitch = asin (2(w·y - z·x))
 *   yaw   = atan2(2(w·z + x·y), 1 - 2(y²+z²))
 *
 * 注意：无磁力计时 yaw 会缓慢漂移，这是物理限制。
 * ===================================================================== */

extern volatile float twoKp;   // 2 * 比例增益 Kp
extern volatile float twoKi;   // 2 * 积分增益 Ki

/* rad → 度的转换系数 */
#define rad_to_angle 57.29578f

/* 带磁力计版本 */
void MahonyAHRSupdate(float q[4],
                      float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float mx, float my, float mz);

/* 仅陀螺仪 + 加速度计版本（常用） */
void MahonyAHRSupdateIMU(float q[4],
                         float gx, float gy, float gz,
                         float ax, float ay, float az);

#endif