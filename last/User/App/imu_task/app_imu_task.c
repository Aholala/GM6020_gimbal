/* app_imu_task.c
 * FreeRTOS IMUTask 实现
 * 负责：BMI088 读取 → 上电校准（Welford）→ AKF → 零偏补偿 → Mahony → PF → 输出欧拉角
 * 运行频率：100Hz（osDelay(10)）
 */

#include "app_imu_task.h"
#include "app_sentry_globals.h"

void StartIMUTask(void *argument)
{
    /* Mahony 四元数初始值：单位四元数（无旋转）*/
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    /* PF 首帧初始化标志（等 Mahony 收敛后再初始化）*/
    uint8_t pf_initialized = 0;

    /* 校准阶段累加变量 */
    double bias_sum[3]  = {0.0, 0.0, 0.0};
    double wf_mean[3]   = {0.0, 0.0, 0.0};
    double wf_M2[3]     = {0.0, 0.0, 0.0};
    double accel_sum[3] = {0.0, 0.0, 0.0};
    int    calib_cnt    = 0;
    const  int calib_total = GYRO_CALIB_MS / 10;  /* 200 次 */

    /* ZUPT 零偏估计状态 */
    float yaw_drift_bias = 0.0f;
    float yaw_drift_intg = 0.0f;

    const float dt = 0.01f;

    for (;;)
    {
        osDelay(10);  /* 100Hz */

        /* ---------------------------------------------------------------- */
        /* 1. BMI088 读取                                                    */
        /* ---------------------------------------------------------------- */
        fp32 gyro[3], accel[3], temp;
        BMI088_read(gyro, accel, &temp);

        /* 原始数据挂 Ozone */
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
        /*    b) 零偏均值（double 累加）                                     */
        /*    c) 校准结束：初始化 AKF，让 Mahony 收敛，记录安装偏移          */
        /* ---------------------------------------------------------------- */
        if (!imu_calib_done)
        {
            /* ---- a) 陀螺零偏累加 ---- */
            bias_sum[0] += (double)gyro[0];
            bias_sum[1] += (double)gyro[1];
            bias_sum[2] += (double)gyro[2];

            /* ---- b) 加速度计累加（校准结束时算安装偏移）---- */
            accel_sum[0] += (double)accel[0];
            accel_sum[1] += (double)accel[1];
            accel_sum[2] += (double)accel[2];

            /* ---- c) Welford 在线方差更新 → AKF 的 R0 ---- */
            calib_cnt++;
            for (int axis = 0; axis < 3; axis++)
            {
                double val    = (double)gyro[axis];
                double delta  = val - wf_mean[axis];
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

                /* ---- 三轴 R0（无偏方差），防止为 0 退化为纯跟随 ---- */
                float r0_gx = (calib_cnt > 1) ? (float)(wf_M2[0] / (calib_cnt - 1)) : 1e-8f;
                float r0_gy = (calib_cnt > 1) ? (float)(wf_M2[1] / (calib_cnt - 1)) : 1e-8f;
                float r0_gz = (calib_cnt > 1) ? (float)(wf_M2[2] / (calib_cnt - 1)) : 1e-8f;
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

                /* ---- 用加速度计 200 次均值计算安装偏移 ---- */
                float ax_mean = (float)(accel_sum[0] / calib_cnt);
                float ay_mean = (float)(accel_sum[1] / calib_cnt);
                float az_mean = (float)(accel_sum[2] / calib_cnt);

                float a_norm = sqrtf(ax_mean*ax_mean + ay_mean*ay_mean + az_mean*az_mean);
                if (a_norm > 0.1f)
                {
                    ax_mean /= a_norm;
                    ay_mean /= a_norm;
                    az_mean /= a_norm;
                }

                float ax0 = ax_mean;
                float ay0 = ay_mean;
                float az0 = az_mean;

                /* 由重力方向计算 roll/pitch 安装偏移 */
                float pitch_offset_raw = -asinf(-ax0) * rad_to_angle;
                float roll_offset_raw  =  atan2f(ay0, az0) * rad_to_angle;
                /* 运行段互换：roll_m = pitch_m, pitch_m = -roll_m */
                euler_offset[0] =  pitch_offset_raw;
                euler_offset[1] = -roll_offset_raw;
                euler_offset[2] = 0.0f;

                /* 用安装偏移初始化 Mahony 四元数，保证后续积分起点正确 */
                {
                    float roll_r  =  euler_offset[0] / rad_to_angle;
                    float pitch_r = -euler_offset[1] / rad_to_angle;
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
            }

            /* 校准期间跳过 IMU 更新 */
            continue;
        }

        /* ---------------------------------------------------------------- */
        /* 3. 正常运行：AKF → 零偏补偿 → Mahony → 欧拉角 → PF            */
        /* ---------------------------------------------------------------- */

        /* ---- 3a. Gyro_AKF 滤波（原始域，减零偏之前）
         *          gz 不经 AKF 直接透传，避免动态压缩                     */
        float gx_akf = Gyro_AKF_Update(&akf_gx, gyro[0]);
        float gy_akf = Gyro_AKF_Update(&akf_gy, gyro[1]);
        float gz_akf = gyro[2];

        dbg_gx_akf = gx_akf;
        dbg_gy_akf = gy_akf;
        dbg_gz_akf = gz_akf;

        /* ---- 3b. 减零偏 ---- */
        float gx = gx_akf - gyro_bias[0];
        float gy = gy_akf - gyro_bias[1];
        float gz = gz_akf - gyro_bias[2];

        /* ---- 3c. 加速度幅值门限：接近 1g 才参与姿态纠正 ---- */
        float accel_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
        float ax, ay, az;
        if (accel_norm >= ACCEL_VALID_MIN && accel_norm <= ACCEL_VALID_MAX)
        {
            ax = accel[0];
            ay = accel[1];
            az = accel[2];
        }
        else
        {
            /* 幅值异常（有线性加速度/冲击），传入全零让 Mahony 跳过加速度纠正 */
            ax = 0.0f; ay = 0.0f; az = 0.0f;
        }

        /* ---- 3d. Yaw 轴 ZUPT：静止时估计 gz 残余零偏并实时补偿 ---- */
        {
            float gyro_sq      = gx*gx + gy*gy + gz*gz;
            uint8_t yaw_static = (gyro_sq < (YAW_STATIC_THR * YAW_STATIC_THR));

            if (yaw_static)
            {
                float gz_err    = gz;
                yaw_drift_intg += YAW_ZUPT_KI * gz_err * dt;
                if      (yaw_drift_intg >  YAW_BIAS_MAX) yaw_drift_intg =  YAW_BIAS_MAX;
                else if (yaw_drift_intg < -YAW_BIAS_MAX) yaw_drift_intg = -YAW_BIAS_MAX;
                yaw_drift_bias  = YAW_ZUPT_KP * gz_err + yaw_drift_intg;
                if      (yaw_drift_bias >  YAW_BIAS_MAX) yaw_drift_bias =  YAW_BIAS_MAX;
                else if (yaw_drift_bias < -YAW_BIAS_MAX) yaw_drift_bias = -YAW_BIAS_MAX;
            }

            gz -= yaw_drift_bias;
        }

        /* ---- 3e. Mahony 姿态融合 ---- */
        MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);

        /* ---- 3f. 四元数 → 欧拉角（ZYX，°，-180~+180）减安装偏移 ---- */
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

            /* 减安装偏移之前先做 pitch/roll 物理轴互换（安装方向决定）*/
            float tmp = roll_m;
            roll_m  = pitch_m;
            pitch_m = -tmp;

            roll_m  -= euler_offset[0];
            pitch_m -= euler_offset[1];
            yaw_m   -= euler_offset[2];

            /* 归一化到 [-180, +180) */
            if      (roll_m  >  180.0f) roll_m  -= 360.0f;
            else if (roll_m  < -180.0f) roll_m  += 360.0f;
            if      (pitch_m >   90.0f) pitch_m  =  90.0f;
            else if (pitch_m <  -90.0f) pitch_m  = -90.0f;
            if      (yaw_m   >  180.0f) yaw_m   -= 360.0f;
            else if (yaw_m   < -180.0f) yaw_m   += 360.0f;
        }

        /* Mahony 直接输出挂 Ozone（PF 之前，用于对比）*/
        dbg_roll_mahony  = roll_m;
        dbg_pitch_mahony = pitch_m;

        /* ---- 3g. 粒子滤波（Pitch / Roll）---- */
        if (!pf_initialized)
        {
            pf_init(&pf, pitch_m, roll_m);
            pf_initialized = 1;
        }
        else
        {
            pf_predict(&pf);
            pf_update(&pf, pitch_m, roll_m);

            float neff = pf_calculate_neff(&pf);
            if (neff < PF_NEFF_THRESH)
            {
                pf_resample(&pf);
            }

            pf_estimate(&pf);
        }

        /* ---- 写入共享数据（SentryTask 读取 pitch_m 用于 USB 上报）---- */
        g_imu.pitch_m = pitch_m;
        g_imu.roll_m  = roll_m;
        g_imu.yaw_m   = yaw_m;

        /* Ozone 观测变量 */
        dbg_pitch = pitch_m - imu_zero_pitch;
        dbg_roll  = roll_m;
        dbg_yaw   = yaw_m  - imu_zero_yaw;

    } /* end for(;;) */
}
