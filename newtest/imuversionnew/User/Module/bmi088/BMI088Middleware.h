#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

/* ===== 非阻塞 DMA 接口 ============================================== */

/* DMA 传输状态 */
typedef enum {
    BMI088_DMA_IDLE = 0,    /* 空闲，可发起新传输 */
    BMI088_DMA_BUSY,        /* DMA 传输中 */
    BMI088_DMA_DONE,        /* 传输完成，数据可读 */
} BMI088_DMA_State_t;

/* 当前正在传输的目标，用于回调中区分是 accel 还是 gyro */
typedef enum {
    BMI088_DMA_TARGET_NONE = 0,
    BMI088_DMA_TARGET_ACCEL,
    BMI088_DMA_TARGET_GYRO,
    BMI088_DMA_TARGET_TEMP,
} BMI088_DMA_Target_t;

/* accel 读取：1 字节命令 + 1 字节 dummy + 6 字节数据 = 8 字节 */
#define BMI088_ACCEL_DMA_LEN  8
/* gyro 读取：从 CHIP_ID 起连读 8 字节（含数据） */
#define BMI088_GYRO_DMA_LEN   8
/* temp 读取：1 字节命令 + 1 字节 dummy + 2 字节数据 = 4 字节 */
#define BMI088_TEMP_DMA_LEN   4

extern volatile BMI088_DMA_State_t  bmi088_dma_state;
extern volatile BMI088_DMA_Target_t bmi088_dma_target;
extern uint8_t bmi088_dma_tx_buf[8];
extern uint8_t bmi088_dma_rx_buf[8];

/*
 * 发起一次 SPI DMA 收发，CS 已在调用前拉低。
 * 传输完成后在 HAL_SPI_TxRxCpltCallback 里拉高 CS 并置 bmi088_dma_state = DONE。
 */
extern void BMI088_spi_dma_start(uint8_t *tx, uint8_t *rx, uint16_t len,
                                 BMI088_DMA_Target_t target);

#endif /* BMI088_USE_SPI */

#endif /* BMI088MIDDLEWARE_H */