/* generated thread header file - do not edit */
#ifndef SCREEN_THREAD_H_
#define SCREEN_THREAD_H_
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hal_data.h"
#ifdef __cplusplus
                extern "C" void Screen_Thread_entry(void * pvParameters);
                #else
extern void Screen_Thread_entry(void *pvParameters);
#endif
FSP_HEADER
FSP_FOOTER
#endif /* SCREEN_THREAD_H_ */
