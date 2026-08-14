#include <Can_Thread.h>
#include "hardware/canfd0.h"
#include "hardware/handeye_transform.h"
#include "ipc_detection_protocol.h"
#include "ipc_detection_rx.h"

typedef enum e_ipc_coordinate_rx_state
{
    IPC_RX_WAIT_BEGIN = 0,
    IPC_RX_READ_BATCH_ID,
    IPC_RX_READ_COUNT,
    IPC_RX_READ_CLASS,
    IPC_RX_READ_TOP_X,
    IPC_RX_READ_TOP_Y,
    IPC_RX_READ_SIDE_X,
    IPC_RX_READ_SIDE_Y,
    IPC_RX_WAIT_END,
} ipc_coordinate_rx_state_t;

#define IPC_RESULT_QUEUE_LENGTH (4U)

typedef struct st_ipc_arm_result
{
    handeye_arm_point_t point;
    uint32_t pair_id;
    uint32_t class_id;
} ipc_arm_result_t;

static ipc_camera_coordinate_batch_t g_receive_batch;
static ipc_camera_coordinate_pair_t g_pair_queue[IPC_RESULT_QUEUE_LENGTH];
static volatile uint32_t g_pair_queue_write;
static volatile uint32_t g_pair_queue_read;
static ipc_arm_result_t g_arm_queue[IPC_RESULT_QUEUE_LENGTH];
static volatile uint32_t g_arm_queue_write;
static volatile uint32_t g_arm_queue_read;

static void ipc_coordinate_rx_reset(ipc_coordinate_rx_state_t * p_state)
{
    *p_state = IPC_RX_WAIT_BEGIN;
}

static void ipc_coordinate_batch_publish(void)
{
    uint32_t const used = (g_pair_queue_write + IPC_RESULT_QUEUE_LENGTH - g_pair_queue_read) %
                          IPC_RESULT_QUEUE_LENGTH;
    uint32_t const free_slots = (IPC_RESULT_QUEUE_LENGTH - 1U) - used;

    if (g_receive_batch.count > free_slots)
    {
        return;
    }

    for (uint32_t i = 0U; i < g_receive_batch.count; i++)
    {
        ipc_camera_coordinate_item_t const * p_item = &g_receive_batch.items[i];
        ipc_camera_coordinate_pair_t * p_pair = &g_pair_queue[g_pair_queue_write];

        p_pair->pair_id = g_receive_batch.batch_id;
        p_pair->class_id = p_item->class_id;
        p_pair->top_x = p_item->top_x;
        p_pair->top_y = p_item->top_y;
        p_pair->side_x = p_item->side_x;
        p_pair->side_y = p_item->side_y;
        g_pair_queue_write = (g_pair_queue_write + 1U) % IPC_RESULT_QUEUE_LENGTH;
    }
}

void ipc0_callback(ipc_callback_args_t *p_args)
{
    static ipc_coordinate_rx_state_t receive_state = IPC_RX_WAIT_BEGIN;
    static uint32_t receive_item_index;

	if ((NULL == p_args) || (IPC_EVENT_MESSAGE_RECEIVED != p_args->event))
	{
		return;
	}

	/*
	 * A new packet header always takes precedence. This lets the receiver
	 * recover when CPU0 times out after sending only part of a packet.
	 */
    if (IPC_COORDINATE_BATCH_BEGIN == p_args->message)
    {
        receive_item_index = 0U;
        receive_state = IPC_RX_READ_BATCH_ID;
        return;
	}

	switch (receive_state)
	{
		case IPC_RX_WAIT_BEGIN:
		{
			break;
		}

        case IPC_RX_READ_BATCH_ID:
        {
            g_receive_batch.batch_id = p_args->message;
            receive_state = IPC_RX_READ_COUNT;
            break;
        }

        case IPC_RX_READ_COUNT:
        {
            if ((0U == p_args->message) ||
                (p_args->message > IPC_COORDINATE_BATCH_MAX_ITEMS))
            {
                ipc_coordinate_rx_reset(&receive_state);
                break;
            }

            g_receive_batch.count = p_args->message;
            receive_state = IPC_RX_READ_CLASS;
            break;
        }

        case IPC_RX_READ_CLASS:
        {
            g_receive_batch.items[receive_item_index].class_id = p_args->message;
            receive_state = IPC_RX_READ_TOP_X;
            break;
        }

        case IPC_RX_READ_TOP_X:
        {
            g_receive_batch.items[receive_item_index].top_x = (int32_t) p_args->message;
            receive_state = IPC_RX_READ_TOP_Y;
            break;
        }

        case IPC_RX_READ_TOP_Y:
        {
            g_receive_batch.items[receive_item_index].top_y = (int32_t) p_args->message;
            receive_state = IPC_RX_READ_SIDE_X;
            break;
        }

        case IPC_RX_READ_SIDE_X:
        {
            g_receive_batch.items[receive_item_index].side_x = (int32_t) p_args->message;
            receive_state = IPC_RX_READ_SIDE_Y;
            break;
        }

        case IPC_RX_READ_SIDE_Y:
        {
            g_receive_batch.items[receive_item_index].side_y = (int32_t) p_args->message;
            receive_item_index++;
            receive_state = (receive_item_index < g_receive_batch.count) ?
                            IPC_RX_READ_CLASS : IPC_RX_WAIT_END;
            break;
        }

		case IPC_RX_WAIT_END:
		{
			if (IPC_COORDINATE_BATCH_END == p_args->message)
            {
                ipc_coordinate_batch_publish();
            }

            ipc_coordinate_rx_reset(&receive_state);
            break;
		}

		default:
		{
            ipc_coordinate_rx_reset(&receive_state);
            break;
		}
	}
}

bool ipc_coordinate_pair_take(ipc_camera_coordinate_pair_t * p_pair)
{
    if (NULL == p_pair)
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (g_pair_queue_read == g_pair_queue_write)
    {
        taskEXIT_CRITICAL();
        return false;
    }

    *p_pair = g_pair_queue[g_pair_queue_read];
    g_pair_queue_read = (g_pair_queue_read + 1U) % IPC_RESULT_QUEUE_LENGTH;

    taskEXIT_CRITICAL();
    return true;
}

bool ipc_arm_point_take(handeye_arm_point_t * p_arm_point,
                        uint32_t            * p_pair_id,
                        uint32_t            * p_class_id)
{
    if ((NULL == p_arm_point) || (NULL == p_pair_id) || (NULL == p_class_id))
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (g_arm_queue_read == g_arm_queue_write)
    {
        taskEXIT_CRITICAL();
        return false;
    }

    *p_arm_point = g_arm_queue[g_arm_queue_read].point;
    *p_pair_id = g_arm_queue[g_arm_queue_read].pair_id;
    *p_class_id = g_arm_queue[g_arm_queue_read].class_id;
    g_arm_queue_read = (g_arm_queue_read + 1U) % IPC_RESULT_QUEUE_LENGTH;

    taskEXIT_CRITICAL();
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
        ipc_camera_coordinate_pair_t pair;

        if (ipc_coordinate_pair_take(&pair))
        {
            handeye_arm_point_t arm_point;

            if (handeye_pixels_to_arm_3d(pair.top_x,
                                         pair.top_y,
                                         pair.side_x,
                                         &arm_point))
            {
                taskENTER_CRITICAL();
                uint32_t const next = (g_arm_queue_write + 1U) % IPC_RESULT_QUEUE_LENGTH;
                if (next != g_arm_queue_read)
                {
                    g_arm_queue[g_arm_queue_write].point = arm_point;
                    g_arm_queue[g_arm_queue_write].pair_id = pair.pair_id;
                    g_arm_queue[g_arm_queue_write].class_id = pair.class_id;
                    g_arm_queue_write = next;
                }
                taskEXIT_CRITICAL();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
	}
}

