#include <Can_Thread.h>
#include "hardware/canfd0.h"
/* New Thread entry function */
/* pvParameters contains TaskHandle_t */
void Can_Thread_entry(void *pvParameters) {
	FSP_PARAMETER_NOT_USED(pvParameters);

	CANFD0_Init();
	CANFD0_Operation_4(10);
	run();

	/* TODO: add your own code here */
//	while (1) {
//		vTaskDelay(1);
//	}
}
