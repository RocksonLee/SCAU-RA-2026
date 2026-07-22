#include <Can_Thread.h>
#include "hardware/canfd0.h"

#define IPC_COORDINATE_HEADER    (0x434F4F52U)

static volatile float g_ipc_pos_x;
static volatile float g_ipc_pos_y;
static volatile bool  g_ipc_coordinate_ready;

void ipc0_callback(ipc_callback_args_t *p_args)
{
 	static uint8_t  receive_state;
	static uint32_t x_bits;

	if (IPC_EVENT_MESSAGE_RECEIVED != p_args->event)
	{
		return;
	}

	if (0U == receive_state)
	{
		if (IPC_COORDINATE_HEADER == p_args->message)
		{
			receive_state = 1U;
		}
	}
	else if (1U == receive_state)
	{
		x_bits       = p_args->message;
		receive_state = 2U;
	}
	else
	{
		union
		{
			uint32_t bits;
			float    value;
		} x, y;

		x.bits = x_bits;
		y.bits = p_args->message;
		g_ipc_pos_x           = x.value;
		g_ipc_pos_y           = y.value;
		g_ipc_coordinate_ready = true;
		receive_state          = 0U;
	}
}

/* New Thread entry function */
/* pvParameters contains TaskHandle_t */
void Can_Thread_entry(void *pvParameters) {
	FSP_PARAMETER_NOT_USED(pvParameters);
	extern int inverse_kinematics_5dof(double x, double y, double z, int elbow_dir,
	                                  double *q1, double *q2, double *q3, double *q5);

	if (FSP_SUCCESS != g_ipc0.p_api->open(g_ipc0.p_ctrl, g_ipc0.p_cfg))
	{
		vTaskDelete(NULL);
	}

	CANFD0_Init();
	while (1)
	{
		if (g_ipc_coordinate_ready)
		{
			float x;
			float y;
			double q1_angle;
			double q2_angle;
			double q3_angle;
			double q5_angle;

			taskENTER_CRITICAL();
			x = g_ipc_pos_x;
			y = g_ipc_pos_y;
			g_ipc_coordinate_ready = false;
			taskEXIT_CRITICAL();

			if (inverse_kinematics_5dof(x, y, 320.0, 1,
			                                &q1_angle, &q2_angle,
			                                &q3_angle, &q5_angle))
			{
				CANFD0_Operation_1((int32_t) q1_angle);
				CANFD0_Operation_2((int32_t) q2_angle);
				CANFD0_Operation_3((int32_t) q3_angle);
				CANFD0_Operation_5((int32_t) q5_angle);
				run();
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
