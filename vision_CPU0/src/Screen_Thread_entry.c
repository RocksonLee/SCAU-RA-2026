#include "Screen_Thread.h"
#include "Touch_Thread.h"

#include "hardware/fruit_ui.h"
#include "hardware/camera_stream.h"
#include "hardware/ipc_detection_tx.h"
#include "hardware/lcd_spi.h"
#include "hardware/lv_port_disp.h"
#include "hardware/lv_port_indev.h"
#include "hardware/ospi_flash.h"
#include "hardware/ui_assets.h"
#include "ipc_detection_protocol.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

extern TaskHandle_t Touch_Thread;

#define ARM_TELEMETRY_POLL_MS (500U)

typedef enum e_arm_telemetry_rx_state
{
	ARM_TELEMETRY_RX_WAIT_BEGIN = 0,
	ARM_TELEMETRY_RX_VALID_MASK,
	ARM_TELEMETRY_RX_ANGLES,
	ARM_TELEMETRY_RX_X,
	ARM_TELEMETRY_RX_Y,
	ARM_TELEMETRY_RX_Z,
	ARM_TELEMETRY_RX_WAIT_END,
	ARM_TELEMETRY_RX_CAL_SEQUENCE,
	ARM_TELEMETRY_RX_CAL_SUCCESS,
	ARM_TELEMETRY_RX_CAL_X,
	ARM_TELEMETRY_RX_CAL_Y,
	ARM_TELEMETRY_RX_CAL_Z,
	ARM_TELEMETRY_RX_CAL_WAIT_END,
} arm_telemetry_rx_state_t;

static volatile uint32_t g_arm_telemetry_pending_valid_mask;
static volatile int32_t g_arm_telemetry_pending_angles[IPC_ARM_TELEMETRY_AXIS_COUNT];
static volatile int32_t g_arm_telemetry_pending_x_0p1mm;
static volatile int32_t g_arm_telemetry_pending_y_0p1mm;
static volatile int32_t g_arm_telemetry_pending_z_0p1mm;
static volatile bool g_arm_telemetry_update_pending;

void ipc0_callback(ipc_callback_args_t * p_args)
{
	static arm_telemetry_rx_state_t state = ARM_TELEMETRY_RX_WAIT_BEGIN;
	static uint32_t valid_mask;
	static int32_t angles[IPC_ARM_TELEMETRY_AXIS_COUNT];
	static int32_t x_0p1mm;
	static int32_t y_0p1mm;
	static int32_t z_0p1mm;
	static uint32_t angle_index;
	static uint32_t calibration_sequence;
	static bool calibration_success;
	static int32_t calibration_x_0p1mm;
	static int32_t calibration_y_0p1mm;
	static int32_t calibration_z_0p1mm;

	if ((NULL == p_args) || (IPC_EVENT_MESSAGE_RECEIVED != p_args->event))
	{
		return;
	}

	if (IPC_ARM_TELEMETRY_BEGIN == p_args->message)
	{
		angle_index = 0U;
		state = ARM_TELEMETRY_RX_VALID_MASK;
		return;
	}

	if (IPC_CALIBRATION_RESULT_BEGIN == p_args->message)
	{
		state = ARM_TELEMETRY_RX_CAL_SEQUENCE;
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

		case ARM_TELEMETRY_RX_CAL_SEQUENCE:
			calibration_sequence = p_args->message;
			state = ARM_TELEMETRY_RX_CAL_SUCCESS;
			break;

		case ARM_TELEMETRY_RX_CAL_SUCCESS:
			calibration_success = (0U != p_args->message);
			state = ARM_TELEMETRY_RX_CAL_X;
			break;

		case ARM_TELEMETRY_RX_CAL_X:
			calibration_x_0p1mm = (int32_t) p_args->message;
			state = ARM_TELEMETRY_RX_CAL_Y;
			break;

		case ARM_TELEMETRY_RX_CAL_Y:
			calibration_y_0p1mm = (int32_t) p_args->message;
			state = ARM_TELEMETRY_RX_CAL_Z;
			break;

		case ARM_TELEMETRY_RX_CAL_Z:
			calibration_z_0p1mm = (int32_t) p_args->message;
			state = ARM_TELEMETRY_RX_CAL_WAIT_END;
			break;

		case ARM_TELEMETRY_RX_CAL_WAIT_END:
			if (IPC_CALIBRATION_RESULT_END == p_args->message)
			{
				camera_calibration_notify_arm_result(calibration_sequence,
				                                     calibration_success,
				                                     calibration_x_0p1mm,
				                                     calibration_y_0p1mm,
				                                     calibration_z_0p1mm);
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
		fruit_ui_set_arm_telemetry(valid_mask,
		                           angles,
		                           (valid_mask & 0x07U) == 0x07U,
		                           x_0p1mm,
		                           y_0p1mm,
		                           z_0p1mm);
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

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5U));
	}
}
