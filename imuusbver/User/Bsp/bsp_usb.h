#ifndef __BSP_USB_H
#define __BSP_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define VISION_FRAME_HEADER   0xA5
#define VISION_RX_FRAME_LEN   11
#define VISION_TX_FRAME_LEN   11

    /**
     * PC → MCU 下行帧（11字节）
     * pitch_rad：目标俯仰角，弧度，世界系绝对值
     * yaw_rad  ：目标偏航角，弧度，相对云台上电零点的增量
     */
    typedef struct __attribute__((packed)) {
        uint8_t header;         /* 0xA5                     */
        float   pitch_rad;      /* 目标 pitch（弧度，世界系）  */
        float   yaw_rad;        /* 目标 yaw  （弧度，相对零点）*/
        uint8_t detected;       /* 1=识别到装甲板            */
        uint8_t checksum;       /* XOR[1-9]               */
    } VisionRxFrame_t;

    extern VisionRxFrame_t g_vision_rx;//调试用

    /**
     * MCU → PC 上行帧（11字节）
     * pitch_rad：当前俯仰角，弧度，IMU Mahony 解算
     * yaw_rad  ：当前偏航角，弧度，编码器差分累积
     */

    typedef struct __attribute__((packed)) {
        uint8_t header;         /* 0xA5                    */
        float   pitch_rad;      /* 当前 pitch（弧度，IMU）   */
        float   yaw_rad;        /* 当前 yaw  （弧度，编码器）*/
        uint8_t mode;           /* 当前工作模式            */
        uint8_t checksum;       /* XOR[1-9]             */
    } VisionTxFrame_t;

    /**
     * @brief 初始化 USB 通信模块
     */

    void BSP_USB_Init(void);

    /**
     * @brief 处理 USB 接收数据帧
     */

    void BSP_USB_Receive(void);

    /**
      * @brief 发送当前状态帧
      * @param pitch_rad 当前 pitch 角（弧度，IMU）
      * @param yaw_rad   当前 yaw 角（弧度，编码器）
      * @param mode      当前工作模式
      */

    void BSP_USB_Send(float pitch_rad, float yaw_rad, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif