/* app_sentry_globals.h
 * 所有任务间共享全局变量的 extern 声明
 */

#ifndef APP_SENTRY_GLOBALS_H
#define APP_SENTRY_GLOBALS_H

#include "app_sentry_common.h"

/* ===== bsp_can.c 导出 =============================================== */
extern moto_info_t motor_info[];

/* ===== bsp_usb.c 导出 ============================================== */
extern volatile uint32_t vis_rx_ok;
extern volatile uint32_t vis_rx_total;
extern volatile uint32_t vis_rx_badlen;
extern volatile uint32_t vis_rx_badhdr;
extern volatile uint32_t vis_rx_badchk;
extern volatile uint32_t vis_rx_lastlen;
extern volatile uint8_t  vis_rx_raw[16];
extern volatile uint32_t vis_rx_raw_len;

/* ===== IMU 共享数据 ================================================= */
extern imu_shared_t g_imu;

/* ===== PID 实例 ===================================================== */
extern pid_struct_t angle_pid2, angle_pid4;
extern pid_struct_t speed_pid2, speed_pid4;

/* ===== 主状态机 ===================================================== */
extern ctrl_state_t state2;
extern ctrl_state_t state4;

/* ===== 扫描轨迹（Yaw/Pitch 独立）=================================== */
extern volatile float SCAN_AMP;
extern volatile float SCAN_FREQ;
extern volatile float SCAN_AMP_PITCH;
extern volatile float SCAN_FREQ_PITCH;

extern float prof_pos;
extern float scan_phase;
extern float prof_pos_pitch;      /* Pitch 轴独立扫描位置 */
extern float scan_phase_pitch;    /* Pitch 轴独立扫描相位 */
extern float dbg_spd_ff;

/* ===== 归零虚拟目标 ================================================= */
extern float homing_target2;
extern float homing_target4;

/* ===== 速度低通 ===================================================== */
extern float filtered_spd2;
extern float filtered_spd4;

/* ===== 编码器零点 =================================================== */
extern uint16_t zero_enc2;
extern uint16_t zero_enc4;

/* ===== 自瞄目标角度 ================================================= */
extern float track_target_yaw;
extern float track_target_pitch;

/* ===== 视觉坐标系偏移 ============================================== */
extern float vis_yaw_offset;
extern float vis_pitch_offset;

/* ===== 视觉低通滤波状态 ============================================= */
extern volatile float vis_yaw_filtered;
extern volatile float vis_pitch_filtered;

/* ===== 回零保护锁 / 丢目标计时 ===================================== */
extern uint8_t  returning_lock;
extern uint32_t lost_timer_ms;

/* ===== 上电视觉暖机计数 ============================================= */
extern volatile uint32_t dbg_vis_warmup_cnt;

/* ===== 阶跃测试模块 ================================================= */
extern volatile uint8_t  step_test_en;
extern volatile float    step_target_yaw;
extern volatile float    step_target_pitch;
extern volatile uint8_t  step_trigger;
extern volatile float    dbg_step_ref2;
extern volatile float    dbg_step_ref4;
extern volatile uint32_t dbg_step_t0_ms;

/* ===== Ozone 观测变量 ============================================== */
extern volatile float   dbg_angle2, dbg_angle4;
extern volatile float   dbg_prof, dbg_prof_pitch;
extern volatile float   dbg_err2, dbg_err4;
extern volatile float   dbg_voltage2, dbg_voltage4;
extern volatile int     dbg_state;
extern volatile fp32    dbg_gyro_x, dbg_gyro_y, dbg_gyro_z;
extern volatile fp32    dbg_accel_x, dbg_accel_y, dbg_accel_z;
extern volatile fp32    dbg_temp;
extern volatile uint8_t dbg_imu_error;
extern volatile float   dbg_gx_akf, dbg_gy_akf, dbg_gz_akf;
extern volatile float   dbg_roll_mahony, dbg_pitch_mahony;
extern volatile float   dbg_roll, dbg_pitch, dbg_yaw, dbg_yaw_enc;
extern volatile int     dbg_calib_samples;
extern volatile float   dbg_r0_gx, dbg_r0_gy, dbg_r0_gz;
extern volatile uint8_t dbg_vis_detected;
extern volatile float   dbg_vis_yaw, dbg_vis_pitch;

/* ===== AKF / PF 实例 =============================================== */
extern Gyro_AKF_HandleTypeDef akf_gx, akf_gy, akf_gz;
extern ParticleFilter_t pf;

/* ===== 陀螺仪零偏 & 安装偏移 ======================================= */
extern float gyro_bias[3];
extern float euler_offset[3];
extern float imu_zero_pitch;
extern float imu_zero_yaw;
extern volatile int imu_calib_done;

/* ===== 辅助函数 ===================================================== */
float enc_to_angle(uint16_t enc, uint16_t zero);
float angle_err_calc(float target, float current);

#endif /* APP_SENTRY_GLOBALS_H */