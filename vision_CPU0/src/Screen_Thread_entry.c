#include "Screen_Thread.h"
#include "Touch_Thread.h"

#include "hardware/fruit_ui.h"
#include "hardware/ipc_detection_tx.h"
#include "hardware/lcd_spi.h"
#include "hardware/lv_port_disp.h"
#include "hardware/lv_port_indev.h"
#include "hardware/ospi_flash.h"
#include "hardware/ui_assets.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

extern TaskHandle_t Touch_Thread;

/* Screen_Thread entry function */
/* pvParameters contains TaskHandle_t */
static uint32_t lvgl_freertos_tick_ms(void)
{
	return (uint32_t) pdTICKS_TO_MS(xTaskGetTickCount());
}

static void screen_process_arm_control_request(void)
{
	uint8_t axis;
	int32_t angle_deg;

	if (fruit_ui_take_arm_zero_request())
	{
		(void) ipc_detection_send_arm_zero();
		return;
	}

	if (fruit_ui_take_axis_angle_request(&axis, &angle_deg))
	{
		bool const sent = ipc_detection_send_axis_angle(axis, angle_deg);
		fruit_ui_notify_axis_angle_result(axis, angle_deg, sent);
	}
}

void Screen_Thread_entry(void *pvParameters)
{
	FSP_PARAMETER_NOT_USED(pvParameters);

	(void) ospi_flash_init();
	(void) ui_assets_init();
	lcd_init();
	lv_init();
	lv_tick_set_cb(lvgl_freertos_tick_ms);
	lv_port_disp_init();
	lv_port_indev_init();
	fruit_ui_create();

	/* Start I2C touch sampling only after FT6336 and LVGL are ready. */
	if (NULL != Touch_Thread)
	{
		xTaskNotifyGive(Touch_Thread);
	}

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1)
	{
		fruit_ui_process();
		(void) lv_timer_handler();
		screen_process_arm_control_request();

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5U));
	}
}
