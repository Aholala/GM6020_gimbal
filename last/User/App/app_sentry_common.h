/* app_sentry_common.h
 * 所有任务共享的宏定义、类型声明
 * 不包含全局变量声明（见 app_sentry_globals.h）
 */

#ifndef APP_SENTRY_COMMON_H
#define APP_SENTRY_COMMON_H

#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "../Bsp/can/bsp_can.h"
#include "../Bsp/usb/bsp_usb.h"
#include "../Bsp/bmi088/bsp_BMI088driver.h"
#include "../Module/pid/module_pid.h"
#include "lib_MahonyAHRS.h"
#include "lib_AKF.h"
#include "lib_Config.h"

/* ===== 类型别名 ===================================================== */
#ifndef fp32
typedef float fp32;
#endif

/* ===== 常量 ========================================================= */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* rad_to_angle 已由 lib_MahonyAHRS.h 定义为宏（57.29578f），此处不再重复定义 */

/* ===== 电机索引 ===================================================== */
#define MOTOR2_IDX  2
#define MOTOR4_IDX  4

/* ===== IMU 参数 ===================================================== */
#define GYRO_CALIB_MS    2000
#define ACCEL_VALID_MIN   5.0f
#define ACCEL_VALID_MAX  15.0f

/* ===== AKF 初始化参数 =============================================== */
#define AKF_P0  1.0f
#define AKF_Q0  1e-7f

/* ===== 粒子滤波参数 ================================================= */
#define PF_NEFF_THRESH  (PF_PARTICLE_NUM / 2.0f)

/* ===== 正弦扫描参数 ================================================= */
#define SCAN_AMP              40.0f
#define SCAN_FREQ              0.5f
#define SPEED_FF_GAIN          0.0f
#define HOMING_SPEED          10.0f
#define RETURN_SPEED          60.0f
#define RETURN_ANGLE_OUT_MAX  80.0f
#define HOMING_THRESH          3.0f

/* ===== PID 参数 ===================================================== */
#define ANGLE_KP        6.0f
#define ANGLE_KI        0.5f
#define ANGLE_KD        0.0f
#define ANGLE_I_MAX    40.0f
#define ANGLE_OUT_MAX 250.0f

#define SPEED_KP      100.0f
#define SPEED_KI        0.05f
#define SPEED_KD        0.0f
#define SPEED_I_MAX  4000.0f
#define SPEED_OUT_MAX 15000.0f

#define LPF_ALPHA  0.5f

/* ===== 自瞄 / 哨兵模式切换参数 ===================================== */
#define TRACK_LIMIT_YAW         60.0f
#define TRACK_LIMIT_PITCH       40.0f
#define LOST_TIMEOUT_MS         500
#define RETURN_THRESH            3.0f
#define TRACK_CONFIRM_FRAMES     1

/* ===== 视觉低通 / 跳变保护 ========================================= */
#define VIS_LPF_ALPHA    0.1f
#define VIS_JUMP_THRESH  10.0f

/* ===== ZUPT 参数 ==================================================== */
#define YAW_STATIC_THR  0.03f
#define YAW_ZUPT_KP     0.02f
#define YAW_ZUPT_KI     0.001f
#define YAW_BIAS_MAX    0.05f

/* ===== 主状态机 ===================================================== */
typedef enum {
    STATE_HOMING = 0,
    STATE_SCAN,
    STATE_RETURNING,
    STATE_TRACK
} ctrl_state_t;

/* ===== IMUTask → SentryTask 共享数据 =============================== */
typedef struct {
    volatile float pitch_m;     /* Mahony pitch（°），SentryTask 用于 USB 上报 */
    volatile float roll_m;      /* Mahony roll（°） */
    volatile float yaw_m;       /* Mahony yaw（°），仅 Ozone 观测 */
    volatile int   calib_done;  /* 校准完成标志 */
} imu_shared_t;

#endif /* APP_SENTRY_COMMON_H */
