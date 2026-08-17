#include "Touch_Thread.h"

#include "hardware/lv_port_indev.h"

#define TOUCH_SAMPLE_PERIOD_MS    5U

/* Touch_Thread entry function */
/* pvParameters contains TaskHandle_t */
void Touch_Thread_entry(void *pvParameters)
{
	FSP_PARAMETER_NOT_USED(pvParameters);

	/* Screen_Thread owns FT6336/LVGL initialization. */
	(void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1)
	{
		lv_port_indev_sample();
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TOUCH_SAMPLE_PERIOD_MS));
	}
}
