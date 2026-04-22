/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "bsp_can.h"

moto_info_t motor_info[MOTOR_MAX_NUM];
uint16_t can_cnt;

/* ===== 诊断变量（全部加入 Ozone Watch）===== */
// [RX 链路]
volatile uint32_t dbg_rx_total      = 0;
volatile uint32_t dbg_rx_can2_match = 0;
volatile uint32_t dbg_rx_id_match   = 0;
volatile uint32_t dbg_rx_last_stdid = 0;

// [ID2 原始字节] 转动电机，观察这些值是否变化
// byte2/3 是速度字段，转动时必须非 0
volatile uint8_t  dbg_raw_byte0 = 0;   // angle  高字节
volatile uint8_t  dbg_raw_byte1 = 0;   // angle  低字节
volatile uint8_t  dbg_raw_byte2 = 0;   // speed  高字节  ← 重点
volatile uint8_t  dbg_raw_byte3 = 0;   // speed  低字节  ← 重点
volatile uint8_t  dbg_raw_byte4 = 0;   // torque 高字节
volatile uint8_t  dbg_raw_byte5 = 0;   // torque 低字节
volatile uint16_t dbg_raw_angle = 0;   // 拼合编码器原始值 0~8191
volatile int16_t  dbg_raw_speed = 0;   // 拼合速度原始值（rpm，有符号）

// [TX 链路]
volatile uint32_t          dbg_tx_total       = 0;
volatile HAL_StatusTypeDef dbg_tx_last_status = HAL_OK; // 0=OK，非0=失败
/* =========================================== */

/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void can_user_init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank           = 14;   // CAN2 过滤器从 bank14 开始
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0;
  can_filter.FilterIdLow          = 0;
  can_filter.FilterMaskIdHigh     = 0;
  can_filter.FilterMaskIdLow      = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation     = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan, &can_filter);
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];

  dbg_rx_total++;

  if (hcan->Instance == CAN2)
  {
    dbg_rx_can2_match++;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    dbg_rx_last_stdid = rx_header.StdId;
  }

  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))
  {
    dbg_rx_id_match++;
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;

    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];

    // 只抓 ID2（index==2）的原始字节
    if (index == 2)
    {
      dbg_raw_byte0 = rx_data[0];
      dbg_raw_byte1 = rx_data[1];
      dbg_raw_byte2 = rx_data[2];   // 转动时应该非 0
      dbg_raw_byte3 = rx_data[3];
      dbg_raw_byte4 = rx_data[4];
      dbg_raw_byte5 = rx_data[5];
      dbg_raw_angle = motor_info[2].rotor_angle;
      dbg_raw_speed = motor_info[2].rotor_speed;
    }
  }
}

void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];

  // GM6020: ID 1~4 用 0x1FF，ID 5~7 用 0x2FF
  tx_header.StdId = (id_range == 0) ? (0x1FF) : (0x2FF);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1 >> 8) & 0xff;
  tx_data[1] =  v1       & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] =  v2       & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] =  v3       & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] =  v4       & 0xff;

  uint32_t tx_mailbox;
  dbg_tx_total++;
  dbg_tx_last_status = HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);
}