/* app_autoaim_task.c
 * FreeRTOS AutoAimTask 实现
 * 负责：USB CDC 接收 → 解析视觉下行帧 → 刷新 g_vision_rx
 * 运行频率：尽可能快（osDelay(1)），视觉帧到达即处理
 */

#include "app_autoaim_task.h"
#include "app_sentry_globals.h"

void StartAutoAimTask(void *argument)
{
    for (;;)
    {
        /* 接收并解析视觉下行帧（内部轮询 USB CDC 接收缓冲区）*/
        BSP_USB_Receive();

        /* 视觉数据挂 Ozone（每帧刷新，不受 10ms 节拍限制）*/
        dbg_vis_detected = g_vision_rx.detected;
        dbg_vis_yaw      = g_vision_rx.yaw_rad;
        dbg_vis_pitch    = g_vision_rx.pitch_rad;

        osDelay(1);  /* 让出 CPU，避免忙等饿死其他任务 */
    }
}
