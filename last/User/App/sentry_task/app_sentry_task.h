/* app_sentry_task.h
 * SentryTask 对外接口声明
 */

#ifndef APP_SENTRY_TASK_H
#define APP_SENTRY_TASK_H

#include "cmsis_os.h"

/* FreeRTOS 任务入口（覆盖 freertos.c 中的 __weak 函数）*/
void StartSentryTask(void *argument);

#endif /* APP_SENTRY_TASK_H */
