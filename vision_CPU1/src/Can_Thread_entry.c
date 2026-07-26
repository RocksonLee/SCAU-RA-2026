#include <Can_Thread.h>
#include "hardware/canfd0.h"
#include "ipc_detection_protocol.h"
#include "ipc_detection_rx.h"

typedef enum e_ipc_detection_rx_state
{
	IPC_RX_WAIT_BEGIN = 0,
	IPC_RX_READ_COUNT,
	IPC_RX_READ_CLASS,
	IPC_RX_READ_X,
	IPC_RX_READ_Y,
	IPC_RX_WAIT_END
} ipc_detection_rx_state_t;

static ipc_detection_result_t g_receive_results[IPC_DETECTION_MAX_RESULTS];
static ipc_detection_result_t g_published_results[IPC_DETECTION_MAX_RESULTS];
static volatile uint32_t       g_published_count;
static volatile bool           g_detection_data_ready;

static void ipc_detection_rx_reset(ipc_detection_rx_state_t * p_state,
								   uint32_t                 * p_result_count,
								   uint32_t                 * p_result_index)
{
	*p_state        = IPC_RX_WAIT_BEGIN;
	*p_result_count = 0U;
	*p_result_index = 0U;
}

static void ipc_detection_publish(uint32_t result_count)
{
	for (uint32_t i = 0U; i < result_count; i++)
	{
		g_published_results[i] = g_receive_results[i];
	}

	g_published_count       = result_count;
	g_detection_data_ready = true;
}

void ipc0_callback(ipc_callback_args_t *p_args)
{
	static ipc_detection_rx_state_t receive_state = IPC_RX_WAIT_BEGIN;
	static uint32_t receive_count;
	static uint32_t receive_index;

	if ((NULL == p_args) || (IPC_EVENT_MESSAGE_RECEIVED != p_args->event))
	{
		return;
	}

	/*
	 * A new packet header always takes precedence. This lets the receiver
	 * recover when CPU0 times out after sending only part of a packet.
	 */
	if (IPC_DETECTION_BEGIN == p_args->message)
	{
		receive_count = 0U;
		receive_index = 0U;
		receive_state = IPC_RX_READ_COUNT;
		return;
	}

	switch (receive_state)
	{
		case IPC_RX_WAIT_BEGIN:
		{
			break;
		}

		case IPC_RX_READ_COUNT:
		{
			receive_count = p_args->message;
			receive_index = 0U;

			if (receive_count > IPC_DETECTION_MAX_RESULTS)
			{
				ipc_detection_rx_reset(&receive_state, &receive_count, &receive_index);
			}
			else
			{
				receive_state = (0U == receive_count) ? IPC_RX_WAIT_END : IPC_RX_READ_CLASS;
			}
			break;
		}

		case IPC_RX_READ_CLASS:
		{
			g_receive_results[receive_index].class_id = p_args->message;
			receive_state = IPC_RX_READ_X;
			break;
		}

		case IPC_RX_READ_X:
		{
			g_receive_results[receive_index].x = (int32_t) p_args->message;
			receive_state = IPC_RX_READ_Y;
			break;
		}

		case IPC_RX_READ_Y:
		{
			g_receive_results[receive_index].y = (int32_t) p_args->message;
			receive_index++;
			receive_state = (receive_index < receive_count) ? IPC_RX_READ_CLASS : IPC_RX_WAIT_END;
			break;
		}

		case IPC_RX_WAIT_END:
		{
			if (IPC_DETECTION_END == p_args->message)
			{
				ipc_detection_publish(receive_count);
			}

			ipc_detection_rx_reset(&receive_state, &receive_count, &receive_index);
			break;
		}

		default:
		{
			ipc_detection_rx_reset(&receive_state, &receive_count, &receive_index);
			break;
		}
	}
}

bool ipc_detection_take_results(ipc_detection_result_t * p_results,
								uint32_t                   result_capacity,
								uint32_t                 * p_result_count)
{
	if ((NULL == p_results) || (NULL == p_result_count))
	{
		return false;
	}

	*p_result_count = 0U;

	taskENTER_CRITICAL();

	if (!g_detection_data_ready)
	{
		taskEXIT_CRITICAL();
		return false;
	}

	uint32_t const available_count = g_published_count;

	if (result_capacity < available_count)
	{
		taskEXIT_CRITICAL();
		*p_result_count = available_count;
		return false;
	}

	for (uint32_t i = 0U; i < available_count; i++)
	{
		p_results[i] = g_published_results[i];
	}

	g_detection_data_ready = false;

	taskEXIT_CRITICAL();

	*p_result_count = available_count;
	return true;
}

/* New Thread entry function */
/* pvParameters contains TaskHandle_t */
void Can_Thread_entry(void *pvParameters) {
	FSP_PARAMETER_NOT_USED(pvParameters);

	if (FSP_SUCCESS != g_ipc0.p_api->open(g_ipc0.p_ctrl, g_ipc0.p_cfg))
	{
		vTaskDelete(NULL);
	}

	//CANFD0_Init();
	while (1)
	{
		/*
		 * Detection results are retained until a target-selection task calls
		 * ipc_detection_take_results(). Camera pixels must be calibrated into
		 * robot coordinates before any motor command is issued.
		 */
		vTaskDelay(pdMS_TO_TICKS(10U));
	}
}
