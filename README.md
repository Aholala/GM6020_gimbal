# GM6020_gimbal
基于stm32f4 二维云台

# GM6020 Gimbal Sentry — FreeRTOS 云台控制固件

RoboMaster 哨兵机器人双轴云台控制固件，基于 STM32F405 + FreeRTOS，集成 BMI088 IMU 姿态解算、GM6020 电机 CAN 控制、USB CDC 视觉通信。

------

## 硬件平台

| 项目     | 型号 / 参数                          |
| -------- | ------------------------------------ |
| MCU      | STM32F405RGT6，168 MHz               |
| IMU      | BMI088（SPI1 + DMA）                 |
| 电机     | GM6020 × 2（ID2 = Yaw，ID4 = Pitch） |
| 总线     | CAN2，1 Mbps                         |
| 视觉通信 | USB CDC（Full Speed OTG）            |
| 调试器   | Segger Ozone（SWD）                  |

------

## 软件架构

```
├── Core/Src/               CubeMX 生成文件（main.c、freertos.c 等）
├── User/
│   ├── App/                应用层（FreeRTOS 任务）
│   │   ├── app_sentry_common.h     共享宏定义与类型声明
│   │   ├── app_sentry_globals.h/c  共享全局变量定义与声明
│   │   ├── app_imu_task.h/c        IMU 任务
│   │   ├── app_autoaim_task.h/c    自瞄任务
│   │   └── app_sentry_task.h/c     哨兵控制任务
│   ├── Bsp/                板级支持层
│   │   ├── bsp_can.h/c             CAN 驱动（GM6020 收发）
│   │   ├── bsp_BMI088driver.h/c    BMI088 驱动
│   │   ├── bsp_usb.h/c             USB CDC 视觉通信
│   │   └── struct_typedef.h        基础类型定义
│   ├── Module/             传感器中间件
│   │   ├── module_BMI088Middleware.h/c  SPI/GPIO 底层操作
│   │   ├── module_BMI088reg.h           寄存器地址定义
│   │   └── module_pid.h/c               PID 控制器
│   └── Lib/                算法库
│       ├── lib_MahonyAHRS.h/c      Mahony 姿态融合
│       ├── lib_AKF.h/c             自适应卡尔曼滤波
│       └── lib_Config.h/c          粒子滤波 & 辅助结构体
```

------

## FreeRTOS 任务

| 任务          | 优先级 | 周期  | 职责                                         |
| ------------- | ------ | ----- | -------------------------------------------- |
| `defaultTask` | Normal | —     | USB Device 初始化                            |
| `IMUTask`     | High   | 10 ms | BMI088 读取 → 校准 → AKF → Mahony → 粒子滤波 |
| `AutoAimTask` | Normal | 1 ms  | USB 接收 → 解析视觉下行帧                    |
| `SentryTask`  | Low    | 10 ms | 状态机 → PID → CAN 发送 → USB 上报           |

`IMUTask` 和 `SentryTask` 均以 100 Hz 运行。`SentryTask` 在 `imu_calib_done` 置位前阻塞等待，保证 IMU 校准完成后才启动电机控制。

------

## IMU 处理流程（IMUTask）

```
BMI088_read()
    │
    ├─ 上电校准（前 2s 静止）
    │     Welford 在线方差 → 三轴 AKF R0
    │     陀螺零偏均值 → gyro_bias[]
    │     加速度计均值 → euler_offset[]（安装偏移）
    │     用安装偏移初始化 Mahony 四元数
    │
    └─ 正常运行
          AKF 滤波（gx/gy，gz 直通）
          减零偏
          加速度幅值门限（5~15 m/s²）
          ZUPT yaw 零偏补偿（PI 估计器）
          MahonyAHRSupdateIMU()
          四元数 → 欧拉角（ZYX）→ 轴互换 → 减安装偏移
          粒子滤波（Pitch / Roll）
          写入 g_imu 共享结构体
```

**关键参数：**

- `GYRO_CALIB_MS = 2000`：校准时长 2 秒
- `sampleFreq = 1000`：Mahony 积分频率（对应 10ms 周期）
- `twoKpDef = 40.0`：Mahony 比例增益
- `twoKiDef = 0.2`：Mahony 积分增益

------

## 状态机（SentryTask）

```
上电
 │
 ▼
STATE_HOMING
 │  两轴虚拟目标以 HOMING_SPEED(10°/s) 步进向 0°
 │  编码器误差 < HOMING_THRESH(3°) → 切 SCAN
 ▼
STATE_SCAN
 │  正弦轨迹：SCAN_AMP(40°) × sin(2π × SCAN_FREQ(0.5Hz) × t)
 │  连续 TRACK_CONFIRM_FRAMES(1) 帧收到视觉 → 切 TRACK
 ▼
STATE_TRACK
 │  视觉低通滤波（VIS_LPF_ALPHA=0.1）
 │  跳变帧保护（VIS_JUMP_THRESH=10°）
 │  超出限位 clamp（Yaw ±60°，Pitch ±40°）
 │  LOST_TIMEOUT_MS(500ms) 无新帧 → 切 RETURNING
 ▼
STATE_RETURNING
 │  以 RETURN_SPEED(60°/s) 步进回零
 │  returning_lock=1（屏蔽视觉切入）
 └─ 两轴均到位 → returning_lock=0 → 切 SCAN
```

------

## CAN 通信

- **总线：** CAN2，PB5（RX）/ PB6（TX），1 Mbps
- **接收：** 电机反馈 ID = `0x204 + motor_index`，FIFO0 中断，优先级 6
- **发送：** GM6020 控制帧 ID = `0x1FF`，槽位顺序对应 ID1~ID4

```c
set_motor_voltage(0, 0, v_yaw, 0, v_pitch);
// 参数顺序：id_range, v_id1, v_id2(yaw), v_id3, v_id4(pitch)
```

------

## USB 视觉通信协议

所有帧均为 **11 字节**，帧头 `0xA5`，校验 XOR[1..9]。

**下行帧（PC → MCU）**

| 字节 | 含义                                 |
| ---- | ------------------------------------ |
| 0    | 帧头 `0xA5`                          |
| 1~4  | `pitch_rad`（float，目标俯仰角，度） |
| 5~8  | `yaw_rad`（float，目标偏航角，度）   |
| 9    | `detected`（0/1）                    |
| 10   | XOR 校验                             |

**上行帧（MCU → PC）**

| 字节 | 含义                                     |
| ---- | ---------------------------------------- |
| 0    | 帧头 `0xA5`                              |
| 1~4  | `pitch_rad`（float，IMU 解算 pitch，度） |
| 5~8  | `yaw_rad`（float，编码器 yaw，度）       |
| 9    | `mode`（`0x01`=扫描/归零，`0x02`=自瞄）  |
| 10   | XOR 校验                                 |

------

## 中断优先级

| 中断                          | 优先级 | 说明                   |
| ----------------------------- | ------ | ---------------------- |
| DMA2 Stream2/3（SPI1 BMI088） | 5      | 最高，IMU 数据不能延迟 |
| CAN2 TX/RX0                   | 6      | 电机控制               |
| USB OTG FS                    | 7      | 视觉通信               |
| TIM7（HAL Tick）              | 15     | HAL 基准时钟           |
| PendSV / SysTick              | 15     | FreeRTOS 调度器        |

`configMAX_SYSCALL_INTERRUPT_PRIORITY = 5`，所有使用 FreeRTOS API 的中断优先级数字均 ≥ 5。

------

## PID 参数

| 参数                | 值                   |
| ------------------- | -------------------- |
| 角度环 Kp / Ki / Kd | 6.0 / 0.5 / 0.0      |
| 角度环积分限幅      | ±40°/s               |
| 角度环输出限幅      | ±250°/s              |
| 速度环 Kp / Ki / Kd | 100.0 / 0.05 / 0.0   |
| 速度环积分限幅      | ±4000                |
| 速度环输出限幅      | ±15000（电机电压值） |
| 速度 LPF α          | 0.5                  |
| 回零输出限幅        | ±80°/s（防甩过）     |

------

## 编译环境

- **IDE：** CLion 2025.3 + CMake
- **工具链：** STM32CubeCLT GNU Arm Embedded（arm-none-eabi-gcc）
- **HAL：** STM32CubeF4 HAL
- **RTOS：** FreeRTOS（CMSIS-RTOS v2）

```bash
cmake --build --target M3508test --preset Debug
```

------

## Ozone 调试变量速查

| 变量                             | 含义                             |
| -------------------------------- | -------------------------------- |
| `dbg_state`                      | 0=归位 1=扫描 2=回零 3=自瞄      |
| `dbg_angle2 / dbg_angle4`        | Yaw / Pitch 编码器角度（°）      |
| `dbg_pitch / dbg_roll / dbg_yaw` | IMU 最终输出欧拉角（°）          |
| `dbg_pitch_mahony`               | Mahony 直接输出（粒子滤波之前）  |
| `dbg_voltage2 / dbg_voltage4`    | Yaw / Pitch 电机电压输出         |
| `dbg_err2 / dbg_err4`            | Yaw / Pitch 角度误差             |
| `dbg_gyro_x/y/z`                 | BMI088 原始陀螺（rad/s）         |
| `dbg_gx/y/z_akf`                 | AKF 滤波后陀螺（rad/s）          |
| `vis_rx_ok`                      | 视觉帧解析成功计数               |
| `dbg_vis_detected`               | 当前帧是否检测到目标             |
| `dbg_calib_samples`              | IMU 校准采样计数（应达到 200）   |
| `dbg_r0_gx/y/z`                  | AKF 三轴量测噪声方差（校准结果） |

------

## 标定说明

**IMU 安装偏移**（上电自动完成）：上电后静置 2 秒，固件自动采集加速度计均值计算安装偏移并存入 `euler_offset[]`，无需手动操作。

**视觉坐标系偏移**（需手动标定）：让云台静止在编码器零点，视觉对准正前方装甲板，在 Ozone 里读取此时 `dbg_vis_yaw` 和 `dbg_vis_pitch` 的值，分别填入 `app_sentry_globals.c` 中的 `vis_yaw_offset` 和 `vis_pitch_offset`。

![image-20260423154405459](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260423154405459.png)

![image-20260423154414947](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260423154414947.png)

![image-20260423154420871](C:\Users\Ahola\AppData\Roaming\Typora\typora-user-images\image-20260423154420871.png)

### 分层设计

整个工程分为四层，职责严格分离：

```
App   ──  FreeRTOS 任务，业务逻辑
Bsp   ──  板级驱动，直接操作硬件
Module──  传感器中间件，寄存器/SPI封装
Lib   ──  纯算法，无硬件依赖
```

### App 层

#### 共享全局变量（`app_sentry_globals`）

三个任务之间没有直接调用，全部通过共享变量通信。定义在 `app_sentry_globals.c`，声明在 `app_sentry_globals.h`，任何任务 include 头文件即可读写：

c

```c
// 任务间共享的 IMU 输出
imu_shared_t g_imu = {0};   // IMUTask 写，SentryTask 读

// 电机状态
ctrl_state_t state2 = STATE_HOMING;   // Yaw 轴状态
ctrl_state_t state4 = STATE_HOMING;   // Pitch 轴状态

// 视觉坐标（AutoAimTask 刷新，SentryTask 消费）
volatile float vis_yaw_filtered   = 0.0f;
volatile float vis_pitch_filtered = 0.0f;
```

#### IMUTask

100 Hz，`osPriorityHigh`。上电前 2 秒做 Welford 在线方差校准，校准完成后每帧走完整信号链：

```c
void StartIMUTask(void *argument)
{
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Mahony 四元数

    for (;;) {
        osDelay(10);  // 100 Hz

        fp32 gyro[3], accel[3], temp;
        BMI088_read(gyro, accel, &temp);

        if (!imu_calib_done) {
            // Welford 在线方差 → 计算 AKF R0 和陀螺零偏
            // 校准完成后 imu_calib_done = 1
            continue;
        }

        // AKF 滤波（gz 直通，避免动态压缩）
        float gx = Gyro_AKF_Update(&akf_gx, gyro[0]) - gyro_bias[0];
        float gy = Gyro_AKF_Update(&akf_gy, gyro[1]) - gyro_bias[1];
        float gz = gyro[2] - gyro_bias[2];

        // ZUPT：静止时 PI 估计 gz 残余零偏
        gz -= yaw_drift_bias;

        // Mahony 姿态融合
        MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);

        // 四元数 → 欧拉角 → 轴互换 → 减安装偏移
        // ...

        // 粒子滤波精化 Pitch/Roll
        pf_predict(&pf);
        pf_update(&pf, pitch_m, roll_m);
        pf_estimate(&pf);

        // 写入共享结构体供 SentryTask 读取
        g_imu.pitch_m = pitch_m;
        g_imu.yaw_m   = yaw_m;
    }
}
```

#### AutoAimTask

1 ms 轮询，`osPriorityNormal`。只做一件事——接收并解析视觉帧：

```c
void StartAutoAimTask(void *argument)
{
    for (;;) {
        BSP_USB_Receive();  // 解析后写入 g_vision_rx

        // 挂 Ozone 观测
        dbg_vis_detected = g_vision_rx.detected;
        dbg_vis_yaw      = g_vision_rx.yaw_rad;
        dbg_vis_pitch    = g_vision_rx.pitch_rad;

        osDelay(1);
    }
}
```

#### SentryTask

100 Hz，`osPriorityLow`。启动前阻塞等待 IMU 校准完成，然后依次执行：读编码器 → 状态机 → PID → CAN 发送 → USB 上报：

```c
void StartSentryTask(void *argument)
{
    // 等 IMU 校准完成才启动电机
    while (!imu_calib_done) osDelay(10);

    for (;;) {
        osDelay(10);

        // 1. 读编码器
        dbg_angle2 = enc_to_angle(motor_info[MOTOR2_IDX].rotor_angle, zero_enc2);
        dbg_angle4 = enc_to_angle(motor_info[MOTOR4_IDX].rotor_angle, zero_enc4);

        // 2. 状态机（见下方）

        // 3. 级联 PID
        // 角度环输出 → 速度目标
        // 速度环输出 → 电机电压
        dbg_voltage2 = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;

        // 4. CAN 发送
        set_motor_voltage(0, 0, (int16_t)dbg_voltage2, 0, (int16_t)dbg_voltage4);

        // 5. USB 上报当前姿态给视觉 PC
        BSP_USB_Send(g_imu.pitch_m, dbg_yaw_enc, tx_mode);
    }
}
```

### 状态机细节

四个状态，切换优先级从高到低：

```c
typedef enum {
    STATE_HOMING = 0,   // 上电归零
    STATE_SCAN,         // 正弦扫描
    STATE_RETURNING,    // 丢目标回零
    STATE_TRACK         // 自瞄跟随
} ctrl_state_t;
```

**视觉活性检测**：用 `vis_rx_ok` 计数器判断本周期是否有新帧，而不是直接读 `detected` 字段。这样视觉断连时会触发丢目标超时，而不是因为 `detected` 保持上一帧值 1 而误认为仍在追踪：

```c
static uint32_t last_vis_rx_ok = 0;
uint8_t vision_active = 0;
if (vis_rx_ok != last_vis_rx_ok) {
    vision_active  = g_vision_rx.detected;
    last_vis_rx_ok = vis_rx_ok;
}
// vis_rx_ok 没变 → 本周期无新帧 → vision_active = 0
```

**切入自瞄时初始化低通滤波器**，防止目标角度跳变导致电机猛冲：

```c
if (track_confirm_cnt >= TRACK_CONFIRM_FRAMES) {
    vis_yaw_filtered   = dbg_angle2;  // 从当前编码器位置出发
    vis_pitch_filtered = dbg_angle4;
    angle_pid2.i_out = 0.0f;          // 清积分
    speed_pid2.i_out = 0.0f;
    state2 = STATE_TRACK;
}
```

**回零保护锁**：`returning_lock = 1` 期间即使视觉重新检测到目标也不切入自瞄，避免回零途中反复抖动：

```c
if (lost_timer_ms >= LOST_TIMEOUT_MS) {
    returning_lock = 1;        // 加锁
    homing_target2 = dbg_angle2;  // 从当前位置出发
    state2 = STATE_RETURNING;
}
// 两轴到位后
returning_lock = 0;            // 解锁
state2 = STATE_SCAN;
```

// 估计重力方向（从四元数推算）
halfvx = q[1]*q[3] - q[0]*q[2];
halfvy = q[0]*q[1] + q[2]*q[3];
halfvz = q[0]*q[0] - 0.5f + q[3]*q[3];

// 叉乘误差：测量重力 × 估计重力
halfex = ay*halfvz - az*halfvy;
halfey = az*halfvx - ax*halfvz;
halfez = ax*halfvy - ay*halfvx;

// 比例 + 积分反馈修正陀螺
gx += twoKp*halfex + integralFBx;
gy += twoKp*halfey + integralFBy;
gz += twoKp*halfez + integralFBz;

// 积分四元数
q[0] += (-q[1]*gx - q[2]*gy - q[3]*gz) * (0.5f/sampleFreq);
// ...归一化



**为什么 gz 不过 AKF？** AKF 会在静止时把 gz 压向零偏均值，动态时滞后明显。Yaw 漂移用 ZUPT（静止零速修正）PI 估计器实时补偿，效果更好。

**为什么用 vis_rx_ok 而不是 detected？** 视觉只在检测到目标时才发帧，`detected` 字段不会自动清零。用帧计数器做活性检测，帧率低时（30 Hz）只有 1/3 的控制周期收到帧，不会被误判为丢失。

**为什么 SentryTask 优先级设最低？** 控制任务 100 Hz 足够，让 IMUTask 和 AutoAimTask 不被抢占，保证数据新鲜度。