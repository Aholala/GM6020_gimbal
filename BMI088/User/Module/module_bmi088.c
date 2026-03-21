#include "module_bmi088.h"

void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8 | buf[0]);
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    bmi088_raw_temp = (int16_t)((buf[3]) << 8 | buf[2]);
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    bmi088_raw_temp = (int16_t)((buf[5]) << 8 | buf[4]);
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8 | buf[2]);
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;

        bmi088_raw_temp = (int16_t)((buf[5]) << 8 | buf[4]);
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;

        bmi088_raw_temp = (int16_t)((buf[7]) << 8 | buf[6]);
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
