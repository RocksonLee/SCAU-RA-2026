/* generated thread header file - do not edit */
#ifndef WEIGHT_THREAD_H_
#define WEIGHT_THREAD_H_
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hal_data.h"
#ifdef __cplusplus
                extern "C" void Weight_Thread_entry(void * pvParameters);
                #else
extern void Weight_Thread_entry(void *pvParameters);
#endif
FSP_HEADER
FSP_FOOTER
#endif /* WEIGHT_THREAD_H_ */
