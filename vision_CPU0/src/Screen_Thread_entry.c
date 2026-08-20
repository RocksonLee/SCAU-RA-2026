#include "Screen_Thread.h"
#include "Touch_Thread.h"

#include <stdio.h>

#include "hardware/fruit_ui.h"
#include "hardware/app_detection.h"
#include "hardware/ipc_detection_tx.h"
#include "hardware/lcd_spi.h"
#include "hardware/lv_port_disp.h"
#include "hardware/lv_port_indev.h"
#include "hardware/ospi_flash.h"
#include "hardware/ui_assets.h"
#include "ipc_detection_protocol.h"
#include "uart_debug.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

extern TaskHandle_t Touch_Thread;

#define ARM_TELEMETRY_POLL_MS (500U)
#define MANUAL_PIXEL_PROBE_TELEMETRY_TIMEOUT_MS (1500U)
#define MANUAL_PIXEL_PROBE_UART_LINE_BYTES       (192U)

typedef enum e_arm_telemetry_rx_state
{
	ARM_TELEMETRY_RX_WAIT_BEGIN = 0,
	ARM_TELEMETRY_RX_VALID_MASK,
	ARM_TELEMETRY_RX_ANGLES,
	ARM_TELEMETRY_RX_X,
	ARM_TELEMETRY_RX_Y,
	ARM_TELEMETRY_RX_Z,
	ARM_TELEMETRY_RX_WAIT_END,
} arm_telemetry_rx_state_t;

static volatile uint32_t g_arm_telemetry_pending_valid_mask;
static volatile int32_t g_arm_telemetry_pending_angles[IPC_ARM_TELEMETRY_AXIS_COUNT];
static volatile int32_t g_arm_telemetry_pending_x_0p1mm;
static volatile int32_t g_arm_telemetry_pending_y_0p1mm;
static volatile int32_t g_arm_telemetry_pending_z_0p1mm;
static volatile bool g_arm_telemetry_update_pending;
static bool g_manual_pixel_probe_report_pending;
static int32_t g_manual_pixel_probe_x;
static int32_t g_manual_pixel_probe_y;
static TickType_t g_manual_pixel_probe_deadline;
static char g_manual_pixel_probe_uart_line[MANUAL_PIXEL_PROBE_UART_LINE_BYTES];
static volatile uint16_t g_performance_ping_sequence;
static volatile TickType_t g_performance_ping_start_tick;
static volatile bool g_performance_ping_pending;

void ipc0_callback(ipc_callback_args_t * p_args)
{
	static arm_telemetry_rx_state_t state = ARM_TELEMETRY_RX_WAIT_BEGIN;
	static uint32_t valid_mask;
	static int32_t angles[IPC_ARM_TELEMETRY_AXIS_COUNT];
	static int32_t x_0p1mm;
	static int32_t y_0p1mm;
	static int32_t z_0p1mm;
	static uint32_t angle_index;

	if ((NULL == p_args) || (IPC_EVENT_MESSAGE_RECEIVED != p_args->event))
	{
		return;
	}

	if (IPC_PERFORMANCE_REPLY_PREFIX ==
	    (p_args->message & IPC_PERFORMANCE_MESSAGE_MASK))
	{
		uint16_t const sequence =
			(uint16_t) (p_args->message & IPC_PERFORMANCE_VALUE_MASK);

		if (g_performance_ping_pending &&
		    (sequence == g_performance_ping_sequence))
		{
			TickType_t const elapsed =
				xTaskGetTickCountFromISR() - g_performance_ping_start_tick;
			uint32_t const elapsed_ms = (uint32_t) pdTICKS_TO_MS(elapsed);

			app_detection_performance_set_ipc_latency(elapsed_ms * 5U);
			g_performance_ping_pending = false;
		}
		return;
	}

	if (IPC_PERFORMANCE_EXEC_ACK_PREFIX ==
	    (p_args->message & IPC_PERFORMANCE_MESSAGE_MASK))
	{
		app_detection_performance_finish_execution(
			p_args->message & IPC_PERFORMANCE_VALUE_MASK);
		return;
	}

	uint32_t const performance_prefix =
		p_args->message & IPC_PERFORMANCE_MESSAGE_MASK;
	if ((IPC_PERFORMANCE_RAM_PREFIX == performance_prefix) ||
	    (IPC_PERFORMANCE_FLASH_PREFIX == performance_prefix) ||
	    (IPC_PERFORMANCE_SDRAM_PREFIX == performance_prefix))
	{
		app_detection_performance_set_cpu1_memory(
			performance_prefix,
			(p_args->message & IPC_PERFORMANCE_VALUE_MASK) * 256U);
		return;
	}

	if (IPC_ARM_TELEMETRY_BEGIN == p_args->message)
	{
		angle_index = 0U;
		state = ARM_TELEMETRY_RX_VALID_MASK;
		return;
	}

	switch (state)
	{
		case ARM_TELEMETRY_RX_VALID_MASK:
			valid_mask = p_args->message;
			state = ARM_TELEMETRY_RX_ANGLES;
			break;

		case ARM_TELEMETRY_RX_ANGLES:
			angles[angle_index++] = (int32_t) p_args->message;
			if (angle_index >= IPC_ARM_TELEMETRY_AXIS_COUNT)
			{
				state = ARM_TELEMETRY_RX_X;
			}
			break;

		case ARM_TELEMETRY_RX_X:
			x_0p1mm = (int32_t) p_args->message;
			state = ARM_TELEMETRY_RX_Y;
			break;

		case ARM_TELEMETRY_RX_Y:
			y_0p1mm = (int32_t) p_args->message;
			state = ARM_TELEMETRY_RX_Z;
			break;

		case ARM_TELEMETRY_RX_Z:
			z_0p1mm = (int32_t) p_args->message;
			state = ARM_TELEMETRY_RX_WAIT_END;
			break;

		case ARM_TELEMETRY_RX_WAIT_END:
			if (IPC_ARM_TELEMETRY_END == p_args->message)
			{
				g_arm_telemetry_pending_valid_mask = valid_mask;
				for (uint32_t i = 0U; i < IPC_ARM_TELEMETRY_AXIS_COUNT; i++)
				{
					g_arm_telemetry_pending_angles[i] = angles[i];
				}
				g_arm_telemetry_pending_x_0p1mm = x_0p1mm;
				g_arm_telemetry_pending_y_0p1mm = y_0p1mm;
				g_arm_telemetry_pending_z_0p1mm = z_0p1mm;
				g_arm_telemetry_update_pending = true;
			}
			state = ARM_TELEMETRY_RX_WAIT_BEGIN;
			break;

		case ARM_TELEMETRY_RX_WAIT_BEGIN:
		default:
			break;
	}
}

/* Screen_Thread entry function */
/* pvParameters contains TaskHandle_t */
static uint32_t lvgl_freertos_tick_ms(void)
{
	return (uint32_t) pdTICKS_TO_MS(xTaskGetTickCount());
}

static void screen_process_performance_ping(void)
{
	static TickType_t last_ping_tick;
	TickType_t const now = xTaskGetTickCount();

	if (!fruit_ui_is_performance_page_active())
	{
		last_ping_tick = 0U;
		return;
	}

	if (g_performance_ping_pending)
	{
		if ((now - g_performance_ping_start_tick) >= pdMS_TO_TICKS(2000U))
		{
			g_performance_ping_pending = false;
		}
		else
		{
			return;
		}
	}

	if ((0U == last_ping_tick) ||
	    ((now - last_ping_tick) >= pdMS_TO_TICKS(500U)))
	{
		g_performance_ping_sequence++;
		g_performance_ping_start_tick = now;
		g_performance_ping_pending = true;
		if (ipc_detection_send_performance_ping(g_performance_ping_sequence))
		{
			last_ping_tick = now;
		}
		else
		{
			g_performance_ping_pending = false;
		}
	}
}

static void screen_process_arm_control_request(void)
{
	static TickType_t claw_current_retry_tick;
	uint8_t axis;
	int32_t angle_deg;
	int32_t x_0p1mm;
	int32_t y_0p1mm;
	int32_t z_0p1mm;
	bool claw_open;
	bool debug_stop_enabled;
	uint16_t claw_current_ma;

	TickType_t const now = xTaskGetTickCount();
	if (((0U == claw_current_retry_tick) ||
	     ((int32_t) (now - claw_current_retry_tick) >= 0)) &&
	    fruit_ui_take_claw_current_request(&claw_current_ma))
	{
		bool const sent = ipc_detection_send_claw_current(claw_current_ma);
		fruit_ui_notify_claw_current_result(claw_current_ma, sent);
		claw_current_retry_tick = sent ? 0U :
		                          now + pdMS_TO_TICKS(1000U);
		return;
	}

	if (fruit_ui_take_arm_debug_stop_request(&debug_stop_enabled))
	{
		bool const sent = ipc_detection_send_arm_debug_stop(debug_stop_enabled);
		fruit_ui_notify_arm_debug_stop_result(debug_stop_enabled, sent);
		return;
	}

	if (fruit_ui_take_arm_zero_request())
	{
		(void) ipc_detection_send_arm_zero();
		return;
	}

	if (fruit_ui_take_claw_request(&claw_open))
	{
		bool const sent = ipc_detection_send_claw(claw_open);
		fruit_ui_notify_claw_result(claw_open, sent);
		return;
	}

	if (fruit_ui_take_manual_coordinate_request(&x_0p1mm,
	                                            &y_0p1mm,
	                                            &z_0p1mm))
	{
		bool const sent = ipc_detection_send_manual_coordinate(x_0p1mm,
		                                                         y_0p1mm,
		                                                         z_0p1mm);
		fruit_ui_notify_manual_coordinate_result(x_0p1mm,
		                                         y_0p1mm,
		                                         z_0p1mm,
		                                         sent);
		return;
	}

	if (fruit_ui_take_axis_angle_request(&axis, &angle_deg))
	{
		bool const sent = ipc_detection_send_axis_angle(axis, angle_deg);
		fruit_ui_notify_axis_angle_result(axis, angle_deg, sent);
	}
}

static void screen_process_arm_telemetry(void)
{
	static TickType_t last_request_tick;
	int32_t angles[IPC_ARM_TELEMETRY_AXIS_COUNT];
	uint32_t valid_mask = 0U;
	int32_t x_0p1mm = 0;
	int32_t y_0p1mm = 0;
	int32_t z_0p1mm = 0;
	bool update_pending;

	taskENTER_CRITICAL();
	update_pending = g_arm_telemetry_update_pending;
	if (update_pending)
	{
		valid_mask = g_arm_telemetry_pending_valid_mask;
		for (uint32_t i = 0U; i < IPC_ARM_TELEMETRY_AXIS_COUNT; i++)
		{
			angles[i] = g_arm_telemetry_pending_angles[i];
		}
		x_0p1mm = g_arm_telemetry_pending_x_0p1mm;
		y_0p1mm = g_arm_telemetry_pending_y_0p1mm;
		z_0p1mm = g_arm_telemetry_pending_z_0p1mm;
		g_arm_telemetry_update_pending = false;
	}
	taskEXIT_CRITICAL();

	if (update_pending)
	{
		bool const coordinate_valid = (valid_mask & 0x07U) == 0x07U;

		fruit_ui_set_arm_telemetry(valid_mask,
		                           angles,
		                           coordinate_valid,
		                           x_0p1mm,
		                           y_0p1mm,
		                           z_0p1mm);

		if (g_manual_pixel_probe_report_pending)
		{
			bool queued = false;
			int count;

			if (coordinate_valid)
			{
				count = snprintf(
					g_manual_pixel_probe_uart_line,
					sizeof(g_manual_pixel_probe_uart_line),
					"PIXEL_FK pixel=(%ld,%ld) xyz_0p1mm=(%ld,%ld,%ld)\r\n",
					(long) g_manual_pixel_probe_x,
					(long) g_manual_pixel_probe_y,
					(long) x_0p1mm, (long) y_0p1mm, (long) z_0p1mm);
				if ((count > 0) &&
				    ((size_t) count < sizeof(g_manual_pixel_probe_uart_line)))
				{
					queued = uart_debug_send_text_always(
						g_manual_pixel_probe_uart_line);
				}
			}
			else
			{
				count = snprintf(g_manual_pixel_probe_uart_line,
				                 sizeof(g_manual_pixel_probe_uart_line),
				                 "PIXEL_FK_ERROR px=(%ld,%ld) valid_mask=0x%02lX\r\n",
				                 (long) g_manual_pixel_probe_x,
				                 (long) g_manual_pixel_probe_y,
				                 (unsigned long) valid_mask);
				if ((count > 0) &&
				    ((size_t) count < sizeof(g_manual_pixel_probe_uart_line)))
				{
					(void) uart_debug_send_text_always(
						g_manual_pixel_probe_uart_line);
				}
			}

			g_manual_pixel_probe_report_pending = false;
			fruit_ui_notify_manual_pixel_probe_uart(queued);
		}
	}

	if (fruit_ui_is_arm_setting_active())
	{
		TickType_t const now = xTaskGetTickCount();
		if ((0U == last_request_tick) ||
		    ((now - last_request_tick) >= pdMS_TO_TICKS(ARM_TELEMETRY_POLL_MS)))
		{
			if (ipc_detection_send_arm_telemetry_request())
			{
				last_request_tick = now;
			}
		}
	}
	else
	{
		last_request_tick = 0U;
	}
}

static void screen_process_manual_pixel_probe(void)
{
	int32_t pixel_x;
	int32_t pixel_y;
	bool found;

	if (fruit_ui_take_manual_pixel_probe_result(&pixel_x,
	                                           &pixel_y,
	                                           &found))
	{
		fruit_ui_notify_manual_pixel_probe_capture(found);
		if (!found)
		{
			return;
		}

		g_manual_pixel_probe_x = pixel_x;
		g_manual_pixel_probe_y = pixel_y;
		g_manual_pixel_probe_report_pending = true;
		g_manual_pixel_probe_deadline =
			xTaskGetTickCount() +
			pdMS_TO_TICKS(MANUAL_PIXEL_PROBE_TELEMETRY_TIMEOUT_MS);

		/* Drop any older periodic response so the UART report uses telemetry
		 * requested after this camera frame was captured. */
		taskENTER_CRITICAL();
		g_arm_telemetry_update_pending = false;
		taskEXIT_CRITICAL();

		if (!ipc_detection_send_arm_telemetry_request())
		{
			g_manual_pixel_probe_report_pending = false;
			(void) uart_debug_send_text_always(
				"PIXEL_FK_ERROR telemetry_request_failed\r\n");
			fruit_ui_notify_manual_pixel_probe_uart(false);
		}
		return;
	}

	if (g_manual_pixel_probe_report_pending &&
	    ((int32_t) (xTaskGetTickCount() -
	                g_manual_pixel_probe_deadline) >= 0))
	{
		g_manual_pixel_probe_report_pending = false;
		(void) uart_debug_send_text_always(
			"PIXEL_FK_ERROR telemetry_timeout\r\n");
		fruit_ui_notify_manual_pixel_probe_uart(false);
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
		screen_process_arm_telemetry();
		screen_process_manual_pixel_probe();
		screen_process_performance_ping();

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5U));
	}
}
