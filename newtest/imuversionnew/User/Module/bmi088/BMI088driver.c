#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"


fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;



#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

bool_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

bool_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}


/* =========================================================================
 * 原有阻塞式读取（保留，init 阶段内部使用）
 * ========================================================================= */
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}


/* =========================================================================
 * 非阻塞 DMA 读取接口
 *
 * 用法（裸机主循环）：
 *
 *   // 1. 每帧调用一次，驱动状态机向前推进
 *   BMI088_read_dma_trigger();
 *
 *   // 2. 当所有三段传输都完成后返回 1，可读取结果
 *   if (BMI088_read_dma_is_ready())
 *   {
 *       BMI088_read_dma_get_data(gyro, accel, &temp);
 *   }
 *
 * 内部流程：
 *   STEP_IDLE
 *     → 拉低 ACCEL_CS，发起 accel DMA（8 字节：cmd + dummy + 6 data）
 *   STEP_ACCEL_WAIT
 *     → 回调到来（DONE）→ 解析 accel，拉低 GYRO_CS，发起 gyro DMA（8 字节）
 *   STEP_GYRO_WAIT
 *     → 回调到来（DONE）→ 解析 gyro，拉低 ACCEL_CS，发起 temp DMA（4 字节）
 *   STEP_TEMP_WAIT
 *     → 回调到来（DONE）→ 解析 temp，进入 STEP_READY
 *   STEP_READY
 *     → 外部调用 BMI088_read_dma_get_data() 取走数据后自动复位为 STEP_IDLE
 * ========================================================================= */

typedef enum {
    STEP_IDLE = 0,
    STEP_ACCEL_WAIT,
    STEP_GYRO_WAIT,
    STEP_TEMP_WAIT,
    STEP_READY,
} BMI088_ReadStep_t;

static volatile BMI088_ReadStep_t s_step = STEP_IDLE;

/* 暂存解析结果，供外部取走 */
static fp32 s_gyro[3];
static fp32 s_accel[3];
static fp32 s_temp;

/*
 * 推进状态机：每帧主循环调用一次。
 * 无阻塞，立即返回。
 */
void BMI088_read_dma_trigger(void)
{
    switch (s_step)
    {
        /* ---- 发起 accel 读取 ---- */
        case STEP_IDLE:
            bmi088_dma_tx_buf[0] = BMI088_ACCEL_XOUT_L | 0x80;  /* read cmd */
            bmi088_dma_tx_buf[1] = 0x55;                         /* dummy byte（accel SPI 协议要求） */
            bmi088_dma_tx_buf[2] = 0x55;
            bmi088_dma_tx_buf[3] = 0x55;
            bmi088_dma_tx_buf[4] = 0x55;
            bmi088_dma_tx_buf[5] = 0x55;
            bmi088_dma_tx_buf[6] = 0x55;
            bmi088_dma_tx_buf[7] = 0x55;
            BMI088_ACCEL_NS_L();
            BMI088_spi_dma_start(bmi088_dma_tx_buf, bmi088_dma_rx_buf,
                                 BMI088_ACCEL_DMA_LEN, BMI088_DMA_TARGET_ACCEL);
            s_step = STEP_ACCEL_WAIT;
            break;

        /* ---- accel DMA 完成 → 解析 → 发起 gyro 读取 ---- */
        case STEP_ACCEL_WAIT:
            if (bmi088_dma_state != BMI088_DMA_DONE)
                break;  /* 还没好，下帧再来 */

            /* rx_buf 布局：[0]=cmd echo，[1]=dummy echo，[2..7]=XL XH YL YH ZL ZH */
            {
                int16_t raw;
                raw = (int16_t)((bmi088_dma_rx_buf[3] << 8) | bmi088_dma_rx_buf[2]);
                s_accel[0] = raw * BMI088_ACCEL_SEN;
                raw = (int16_t)((bmi088_dma_rx_buf[5] << 8) | bmi088_dma_rx_buf[4]);
                s_accel[1] = raw * BMI088_ACCEL_SEN;
                raw = (int16_t)((bmi088_dma_rx_buf[7] << 8) | bmi088_dma_rx_buf[6]);
                s_accel[2] = raw * BMI088_ACCEL_SEN;
            }

            /* 发起 gyro 读取：从 CHIP_ID(0x00) 开始连读 8 字节 */
            bmi088_dma_tx_buf[0] = BMI088_GYRO_CHIP_ID | 0x80;
            bmi088_dma_tx_buf[1] = 0x55;
            bmi088_dma_tx_buf[2] = 0x55;
            bmi088_dma_tx_buf[3] = 0x55;
            bmi088_dma_tx_buf[4] = 0x55;
            bmi088_dma_tx_buf[5] = 0x55;
            bmi088_dma_tx_buf[6] = 0x55;
            bmi088_dma_tx_buf[7] = 0x55;
            bmi088_dma_state = BMI088_DMA_IDLE;
            BMI088_GYRO_NS_L();
            BMI088_spi_dma_start(bmi088_dma_tx_buf, bmi088_dma_rx_buf,
                                 BMI088_GYRO_DMA_LEN, BMI088_DMA_TARGET_GYRO);
            s_step = STEP_GYRO_WAIT;
            break;

        /* ---- gyro DMA 完成 → 解析 → 发起 temp 读取 ---- */
        case STEP_GYRO_WAIT:
            if (bmi088_dma_state != BMI088_DMA_DONE)
                break;

            /* rx_buf[0]=CHIP_ID，rx_buf[1]=reserved，rx_buf[2..7]=XL XH YL YH ZL ZH */
            if (bmi088_dma_rx_buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                int16_t raw;
                raw = (int16_t)((bmi088_dma_rx_buf[3] << 8) | bmi088_dma_rx_buf[2]);
                s_gyro[0] = raw * BMI088_GYRO_SEN;
                raw = (int16_t)((bmi088_dma_rx_buf[5] << 8) | bmi088_dma_rx_buf[4]);
                s_gyro[1] = raw * BMI088_GYRO_SEN;
                raw = (int16_t)((bmi088_dma_rx_buf[7] << 8) | bmi088_dma_rx_buf[6]);
                s_gyro[2] = raw * BMI088_GYRO_SEN;
            }

            /* 发起 temp 读取：cmd + dummy + 2 字节数据，共 4 字节 */
            bmi088_dma_tx_buf[0] = BMI088_TEMP_M | 0x80;
            bmi088_dma_tx_buf[1] = 0x55;  /* dummy */
            bmi088_dma_tx_buf[2] = 0x55;
            bmi088_dma_tx_buf[3] = 0x55;
            bmi088_dma_state = BMI088_DMA_IDLE;
            BMI088_ACCEL_NS_L();
            BMI088_spi_dma_start(bmi088_dma_tx_buf, bmi088_dma_rx_buf,
                                 BMI088_TEMP_DMA_LEN, BMI088_DMA_TARGET_TEMP);
            s_step = STEP_TEMP_WAIT;
            break;

        /* ---- temp DMA 完成 → 解析 → 数据就绪 ---- */
        case STEP_TEMP_WAIT:
            if (bmi088_dma_state != BMI088_DMA_DONE)
                break;

            /* rx_buf 布局：[0]=cmd echo，[1]=dummy echo，[2]=TEMP_M，[3]=TEMP_L */
            {
                int16_t raw = (int16_t)((bmi088_dma_rx_buf[2] << 3) |
                                        (bmi088_dma_rx_buf[3] >> 5));
                if (raw > 1023)
                    raw -= 2048;
                s_temp = raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
            }

            bmi088_dma_state = BMI088_DMA_IDLE;
            s_step = STEP_READY;
            break;

        case STEP_READY:
            /* 等待外部取走，不做任何事 */
            break;

        default:
            break;
    }
}

/*
 * 查询本帧数据是否就绪（三段 DMA 全部完成）。
 * 返回 1 表示可以调用 BMI088_read_dma_get_data()。
 */
uint8_t BMI088_read_dma_is_ready(void)
{
    return (s_step == STEP_READY) ? 1u : 0u;
}

/*
 * 取走本帧数据，同时将状态机复位为 STEP_IDLE，供下一帧使用。
 */
void BMI088_read_dma_get_data(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    gyro[0]    = s_gyro[0];
    gyro[1]    = s_gyro[1];
    gyro[2]    = s_gyro[2];
    accel[0]   = s_accel[0];
    accel[1]   = s_accel[1];
    accel[2]   = s_accel[2];
    *temperate = s_temp;

    /* 复位，允许下一帧触发 */
    s_step = STEP_IDLE;
}


#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif