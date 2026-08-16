#include <Can_Thread.h>
#include "hardware/canfd0.h"
#include "hardware/handeye_transform.h"
#include "hardware/jiesuan.h"
#include "ipc_detection_protocol.h"
#include "ipc_detection_rx.h"
#include "uart_coordinate_protocol.h"

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
#define ARM_IK_ELBOW_DIRECTION  (1)
#define ARM_JOINT_2_MAX_DEG     (10.0)
#define ARM_JOINT_5_OFFSET_DEG  (-80.0)
#define ARM_JOINT_5_DELAY_MS    (2000U)

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

#if DM_COORDINATE_UART_ENABLE
extern TaskHandle_t Uart_dm_thread;
#endif

#if !DM_COORDINATE_UART_ENABLE
static bool arm_move_to_point(handeye_arm_point_t const * p_point)
{
    double q1;
    double q2;
    double q3;
    double q5;

    if ((NULL == p_point) ||
        !inverse_kinematics_5dof(p_point->x_mm,
                                 p_point->y_mm,
                                 p_point->z_mm,
                                 ARM_IK_ELBOW_DIRECTION,
                                 &q1,
                                 &q2,
                                 &q3,
                                 &q5))
    {
        return false;
    }

    /*
     * Each position command uses sync flag 1. Queue joints 1 to 3 first and
     * trigger the first-stage motion together. Joint 4 is mechanically locked
     * and receives no CAN command. After the first stage has had time to
     * settle, offset joint 5 by -80 degrees and trigger it separately.
     */
    if (q2 > ARM_JOINT_2_MAX_DEG)
    {
        q2 = ARM_JOINT_2_MAX_DEG;
    }

    CANFD0_Operation_1((int32_t) q1);
    CANFD0_Operation_2((int32_t) q2);
    CANFD0_Operation_3((int32_t) q3);
    run();

    vTaskDelay(pdMS_TO_TICKS(ARM_JOINT_5_DELAY_MS));

    CANFD0_Operation_5((int32_t) (ARM_JOINT_5_OFFSET_DEG + q5));
    run();
    return true;
}
#endif

#if DM_COORDINATE_UART_ENABLE
static bool ipc_arm_point_publish(handeye_arm_point_t const * p_arm_point,
                                  uint32_t                    pair_id,
                                  uint32_t                    class_id)
{
    bool published = false;

    if (NULL == p_arm_point)
    {
        return false;
    }

    taskENTER_CRITICAL();

    uint32_t const next = (g_arm_queue_write + 1U) % IPC_RESULT_QUEUE_LENGTH;
    if (next != g_arm_queue_read)
    {
        g_arm_queue[g_arm_queue_write].point = *p_arm_point;
        g_arm_queue[g_arm_queue_write].pair_id = pair_id;
        g_arm_queue[g_arm_queue_write].class_id = class_id;
        g_arm_queue_write = next;
        published = true;
    }

    taskEXIT_CRITICAL();
    return published;
}
#endif

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

#if !DM_COORDINATE_UART_ENABLE
	CANFD0_Init();
#endif
	while (1)
	{
        ipc_camera_coordinate_pair_t pair;

        if (ipc_coordinate_pair_take(&pair))
        {
            handeye_arm_point_t arm_point;
            bool const top_only = (0 == pair.side_x) && (0 == pair.side_y);

            if ((top_only && handeye_pixel_to_arm(pair.top_x, pair.top_y, &arm_point)) ||
                (!top_only && handeye_pixels_to_arm_3d(pair.top_x,
                                                       pair.top_y,
                                                       pair.side_x,
                                                       &arm_point)))
            {
#if DM_COORDINATE_UART_ENABLE
                if (ipc_arm_point_publish(&arm_point, pair.pair_id, pair.class_id))
                {
                    xTaskNotifyGive(Uart_dm_thread);
                }
#else
                (void) arm_move_to_point(&arm_point);
#endif
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
	}
}

