#include "module_BMI088Middleware.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{
    /* GPIO 已由 MX_GPIO_Init() 完成初始化，此处留空 */
}

void BMI088_com_init(void)
{
    /* SPI1 已由 MX_SPI1_Init() 完成初始化，此处留空 */
}

void BMI088_delay_ms(uint16_t ms)
{
    while (ms--)
    {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{
    /* 基于 SysTick 的精确微秒延时，适配 168 MHz 主频 */
    uint32_t ticks = 0;
    uint32_t told  = 0;
    uint32_t tnow  = 0;
    uint32_t tcnt  = 0;
    uint32_t reload = 0;

    reload = SysTick->LOAD;
    ticks  = (uint32_t)us * 168u;   /* 168 MHz：1 µs = 168 个计数 */
    told   = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break;
        }
    }
}

/* ===== 加速度计片选：CS1 = PC4 ===================================== */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}

/* ===== 陀螺仪片选：CS2 = PB1 ======================================= */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
}

/* ===== SPI1 单字节读写 ============================================== */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}