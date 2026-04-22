#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

extern uint8_t  UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t  UserRxFlag;
extern uint32_t UserRxLen;

/* 解析后的视觉下行数据，供上层模块读取 */
VisionRxFrame_t g_vision_rx = {0};

/* ── 调试用内部函数：通过USB CDC发送ASCII字符串 ───────────────────────── */
static void debug_print(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    /* TxState忙时最多重试3次，每次等1ms，避免丢调试信息 */
    for (int i = 0; i < 3; i++)
    {
        if (CDC_Transmit_FS((uint8_t *)str, len) == 0) /* USBD_OK = 0 */
            break;
        HAL_Delay(1);
    }
}

/**
 * @brief 计算校验和（XOR[1..len-2]）
 */
static uint8_t calc_checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t chk = 0;
    for (uint8_t i = 1; i < len - 1; i++) chk ^= buf[i];
    return chk;
}

/**
 * @brief 初始化 USB 通信模块
 */
void BSP_USB_Init(void)
{
    UserRxFlag = 0;
    UserRxLen  = 0;
    memset(&g_vision_rx, 0, sizeof(g_vision_rx));
}

/**
 * @brief 处理 USB 接收数据帧，调试版：解析成功/失败都打印
 */
void BSP_USB_Receive(void)
{
    char dbg[80];

    if (!UserRxFlag)
        return;
    UserRxFlag = 0;

    /* 长度校验 */
    if (UserRxLen != VISION_RX_FRAME_LEN)
    {
        snprintf(dbg, sizeof(dbg), "[RX FAIL] bad len=%lu\r\n", UserRxLen);
        debug_print(dbg);
        return;
    }

    /* 帧头校验 */
    if (UserRxBufferFS[0] != VISION_FRAME_HEADER)
    {
        snprintf(dbg, sizeof(dbg), "[RX FAIL] bad header=0x%02X\r\n", UserRxBufferFS[0]);
        debug_print(dbg);
        return;
    }

    /* 校验和校验 */
    uint8_t chk = calc_checksum(UserRxBufferFS, VISION_RX_FRAME_LEN);
    if (chk != UserRxBufferFS[VISION_RX_FRAME_LEN - 1])
    {
        snprintf(dbg, sizeof(dbg), "[RX FAIL] bad chk=0x%02X expect=0x%02X\r\n",
                 UserRxBufferFS[VISION_RX_FRAME_LEN - 1], chk);
        debug_print(dbg);
        return;
    }

    /* 解析成功 */
    memcpy(&g_vision_rx, UserRxBufferFS, sizeof(VisionRxFrame_t));

    snprintf(dbg, sizeof(dbg), "[RX OK] pitch=%.4f yaw=%.4f det=%d\r\n",
             g_vision_rx.pitch_rad, g_vision_rx.yaw_rad, g_vision_rx.detected);
    debug_print(dbg);
}

/**
 * @brief 发送当前云台状态帧
 * @note  先发二进制帧给视觉，再打印ASCII确认给串口助手
 */
void BSP_USB_Send(float pitch_rad, float yaw_rad, uint8_t mode)
{
    VisionTxFrame_t frame;
    char dbg[80];

    frame.header    = VISION_FRAME_HEADER;
    frame.pitch_rad = pitch_rad;
    frame.yaw_rad   = yaw_rad;
    frame.mode      = mode;
    frame.checksum  = calc_checksum((const uint8_t *)&frame, VISION_TX_FRAME_LEN);

    /* 发二进制帧给视觉 */
    uint8_t ret = CDC_Transmit_FS((uint8_t *)&frame, VISION_TX_FRAME_LEN);

    /* 等待USB把二进制帧真正发完，最多等10ms */
    extern USBD_HandleTypeDef hUsbDeviceFS;
    uint32_t t = HAL_GetTick();
    while (((USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData)->TxState != 0)
    {
        if (HAL_GetTick() - t > 10) break;
    }

    snprintf(dbg, sizeof(dbg), "[TX %s] pitch=%.4f yaw=%.4f mode=0x%02X\r\n",
             ret == 0 ? "OK" : "BUSY", pitch_rad, yaw_rad, mode);
    debug_print(dbg);
}