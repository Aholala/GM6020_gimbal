/* app_imu_task.h
 * IMUTask 对外接口声明
 */

#ifndef APP_IMU_TASK_H
#define APP_IMU_TASK_H

#include "cmsis_os.h"

/* FreeRTOS 任务入口（覆盖 freertos.c 中的 __weak 函数）*/
void StartIMUTask(void *argument);

#endif /* APP_IMU_TASK_H */
