/* generated thread header file - do not edit */
#ifndef UART9_THREAD_H_
#define UART9_THREAD_H_
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hal_data.h"
#ifdef __cplusplus
                extern "C" void Uart9_thread_entry(void * pvParameters);
                #else
extern void Uart9_thread_entry(void *pvParameters);
#endif
FSP_HEADER
FSP_FOOTER
#endif /* UART9_THREAD_H_ */
