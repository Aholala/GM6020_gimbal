/* app_sentry_globals.c
 * 所有任务间共享全局变量的定义
 */

#include "app_sentry_globals.h"

/* ===== IMU 共享数据 ================================================= */
imu_shared_t g_imu = {0};

/* ===== PID 实例 ===================================================== */
pid_struct_t angle_pid2, angle_pid4;
pid_struct_t speed_pid2, speed_pid4;

/* ===== 主状态机 ===================================================== */
ctrl_state_t state2 = STATE_HOMING;
ctrl_state_t state4 = STATE_HOMING;

/* ===== 扫描轨迹（Yaw/Pitch 独立）=================================== */
/* volatile 允许 Ozone 运行时修改扫描参数 */
volatile float SCAN_AMP        = SCAN_AMP_DEFAULT;
volatile float SCAN_FREQ       = SCAN_FREQ_DEFAULT;
volatile float SCAN_AMP_PITCH  = SCAN_AMP_PITCH_DEFAULT;
volatile float SCAN_FREQ_PITCH = SCAN_FREQ_PITCH_DEFAULT;

float prof_pos        = 0.0f;
float scan_phase      = 0.0f;
float prof_pos_pitch  = 0.0f;
float scan_phase_pitch= 0.0f;
float dbg_spd_ff      = 0.0f;

/* ===== 归零虚拟目标 ================================================= */
float homing_target2 = 0.0f;
float homing_target4 = 0.0f;

/* ===== 速度低通 ===================================================== */
float filtered_spd2 = 0.0f;
float filtered_spd4 = 0.0f;

/* ===== 编码器零点 =================================================== */
uint16_t zero_enc2 = 0;
uint16_t zero_enc4 = 0;

/* ===== 自瞄目标角度（°）=========================================== */
float track_target_yaw   = 0.0f;
float track_target_pitch = 0.0f;

/* ===== 视觉坐标系偏移（Ozone 标定）================================= */
float vis_yaw_offset   = 0.0f;
float vis_pitch_offset = 0.0f;

/* ===== 视觉低通滤波状态 ============================================= */
volatile float vis_yaw_filtered   = 0.0f;
volatile float vis_pitch_filtered = 0.0f;

/* ===== 回零保护锁 / 丢目标计时 ===================================== */
uint8_t  returning_lock = 0;
uint32_t lost_timer_ms  = 0;

/* ===== 上电视觉暖机计数 ============================================= */
volatile uint32_t dbg_vis_warmup_cnt = 0;

/* ===== 阶跃测试模块 ================================================= */
volatile uint8_t  step_test_en      = 0;
volatile float    step_target_yaw   = 5.0f;
volatile float    step_target_pitch = 0.0f;
volatile uint8_t  step_trigger      = 0;
volatile float    dbg_step_ref2     = 0.0f;
volatile float    dbg_step_ref4     = 0.0f;
volatile uint32_t dbg_step_t0_ms    = 0;

/* ===== Ozone 观测变量 ============================================== */
volatile float   dbg_angle2        = 0.0f;
volatile float   dbg_angle4        = 0.0f;
volatile float   dbg_prof          = 0.0f;
volatile float   dbg_prof_pitch    = 0.0f;
volatile float   dbg_err2          = 0.0f;
volatile float   dbg_err4          = 0.0f;
volatile float   dbg_voltage2      = 0.0f;
volatile float   dbg_voltage4      = 0.0f;
volatile int     dbg_state         = 0;

volatile fp32    dbg_gyro_x        = 0.0f;
volatile fp32    dbg_gyro_y        = 0.0f;
volatile fp32    dbg_gyro_z        = 0.0f;
volatile fp32    dbg_accel_x       = 0.0f;
volatile fp32    dbg_accel_y       = 0.0f;
volatile fp32    dbg_accel_z       = 0.0f;
volatile fp32    dbg_temp          = 0.0f;
volatile uint8_t dbg_imu_error     = 0;

volatile float   dbg_gx_akf        = 0.0f;
volatile float   dbg_gy_akf        = 0.0f;
volatile float   dbg_gz_akf        = 0.0f;

volatile float   dbg_roll_mahony   = 0.0f;
volatile float   dbg_pitch_mahony  = 0.0f;
volatile float   dbg_roll          = 0.0f;
volatile float   dbg_pitch         = 0.0f;
volatile float   dbg_yaw           = 0.0f;
volatile float   dbg_yaw_enc       = 0.0f;

volatile int     dbg_calib_samples = 0;
volatile float   dbg_r0_gx         = 0.0f;
volatile float   dbg_r0_gy         = 0.0f;
volatile float   dbg_r0_gz         = 0.0f;

volatile uint8_t dbg_vis_detected  = 0;
volatile float   dbg_vis_yaw       = 0.0f;
volatile float   dbg_vis_pitch     = 0.0f;

/* ===== AKF / PF 实例 =============================================== */
Gyro_AKF_HandleTypeDef akf_gx, akf_gy, akf_gz;
ParticleFilter_t pf;

/* ===== 陀螺仪零偏 & 安装偏移 ======================================= */
float gyro_bias[3]    = {0.0f, 0.0f, 0.0f};
float euler_offset[3] = {0.0f, 0.0f, 0.0f};
float imu_zero_pitch  = 0.0f;
float imu_zero_yaw    = 0.0f;
volatile int imu_calib_done = 0;

/* ===== 辅助函数定义 ================================================= */
float enc_to_angle(uint16_t enc, uint16_t zero)
{
    int32_t diff = (int32_t)enc - (int32_t)zero;
    if      (diff >  4096) diff -= 8192;
    else if (diff < -4096) diff += 8192;
    return (float)diff * 360.0f / 8192.0f;
}

float angle_err_calc(float target, float current)
{
    float e = target - current;
    if      (e >  180.0f) e -= 360.0f;
    else if (e < -180.0f) e += 360.0f;
    return e;
}