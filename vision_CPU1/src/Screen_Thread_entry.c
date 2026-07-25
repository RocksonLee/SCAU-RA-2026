#include "Screen_Thread.h"

#include "hardware/fruit_ui.h"
#include "hardware/lcd_spi.h"
#include "hardware/lv_port_disp.h"
#include "hardware/lv_port_indev.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

/* Screen_Thread entry function */
/* pvParameters contains TaskHandle_t */
void Screen_Thread_entry(void *pvParameters)
{
	FSP_PARAMETER_NOT_USED(pvParameters);

	lcd_init();
	lv_init();
	lv_port_disp_init();
	lv_port_indev_init();
	fruit_ui_create();

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1)
	{
		lv_tick_inc(5U);
		(void) lv_timer_handler();

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5U));
	}
}
