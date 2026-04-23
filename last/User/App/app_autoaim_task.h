/* app_autoaim_task.h
 * AutoAimTask 对外接口声明
 */

#ifndef APP_AUTOAIM_TASK_H
#define APP_AUTOAIM_TASK_H

#include "cmsis_os.h"

/* FreeRTOS 任务入口（覆盖 freertos.c 中的 __weak 函数）*/
void StartAutoAimTask(void *argument);

#endif /* APP_AUTOAIM_TASK_H */
