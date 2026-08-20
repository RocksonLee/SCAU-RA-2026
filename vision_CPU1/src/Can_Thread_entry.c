#include <Can_Thread.h>
#include <math.h>
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
    IPC_RX_READ_MANUAL_X,
    IPC_RX_READ_MANUAL_Y,
    IPC_RX_READ_MANUAL_Z,
    IPC_RX_WAIT_MANUAL_END,
} ipc_coordinate_rx_state_t;

#define IPC_RESULT_QUEUE_LENGTH (4U)
#define ARM_IK_ELBOW_DIRECTION  (1)
#define ARM_JOINT_2_MAX_DEG     (10.0)
#define ARM_JOINT_SETTLE_DELAY_MS (2000U)
#define ARM_CLAW_TIMEOUT_MS      (8000U)
#define ARM_CLAW_SETTLE_DELAY_MS (3000U)
#define ARM_CLAW_OPEN_DELAY_MS   (3000U)
#define ARM_RESET_ZERO_DELAY_MS  (500U)
#define ARM_DROP_AXIS_1_DEG      (0)
#define ARM_DROP_AXIS_2_DEG      (-40)
#define ARM_DROP_AXIS_3_DEG      (15)
#define ARM_DROP_AXIS_4_DEG      (0)
#define ARM_DROP_AXIS_5_DEG      (50)
#define ARM_TASK1_JOINT_5_DEG   (-30)
#define ARM_TASK2_JOINT_5_DEG   (80)
#define ARM_AXIS_COUNT           (5U)
#define ARM_AXIS_ANGLE_QUEUE_LENGTH (16U)
#define ARM_TELEMETRY_IPC_TIMEOUT_MS (100U)
#define ARM_TELEMETRY_IPC_GAP_MS     (2U)
#define ARM_DEG_TO_RAD                (0.017453292519943295)

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
static volatile bool g_arm_zero_request_pending;
static volatile bool g_claw_request_pending;
static volatile bool g_claw_open_requested;
static volatile bool g_task_joint5_request_pending;
static volatile int32_t g_task_joint5_angle_deg;
static volatile bool g_arm_debug_stop_at_target;
static volatile uint32_t g_axis_angle_queue[ARM_AXIS_ANGLE_QUEUE_LENGTH];
static volatile uint32_t g_axis_angle_queue_write;
static volatile uint32_t g_axis_angle_queue_read;
static volatile bool g_arm_telemetry_request_pending;
static volatile ipc_manual_coordinate_t g_manual_coordinate_request;
static volatile bool g_manual_coordinate_request_pending;
static double g_arm_solved_angles_deg[IPC_ARM_TELEMETRY_AXIS_COUNT];
static uint32_t g_arm_solved_angle_valid_mask = 0x1FU;

#if DM_COORDINATE_UART_ENABLE
extern TaskHandle_t Uart_dm_thread;
#endif

#if !DM_COORDINATE_UART_ENABLE
static void arm_publish_telemetry(void);
static bool arm_move_to_zero(void);
static bool arm_prepare_task(int32_t joint5_angle_deg);
static bool arm_return_to_task_pose(void);

static bool arm_move_to_point(handeye_arm_point_t const * p_point)
{
    double q1;
    double q2;
    double q3;
    double q5;
    double solved_q2;

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

    /* Keep the original IK result for telemetry/forward kinematics. The
     * mechanical q2 limit below only changes the value sent to the motor. */
    solved_q2 = q2;

    /*
     * Each position command uses sync flag 1. Queue joints 1 to 3 first and
     * trigger the first-stage motion together. Joint 4 is mechanically locked
     * and receives no CAN command. After the first stage has had time to
     * settle, send the inverse-kinematics joint 5 angle without an offset and
     * trigger it separately.
     */
    if (q2 > ARM_JOINT_2_MAX_DEG)
    {
        q2 = ARM_JOINT_2_MAX_DEG;
    }

    if (!CANFD0_Operation_1((int32_t) round(q1)) ||
        !CANFD0_Operation_2((int32_t) round(q2)) ||
        !CANFD0_Operation_3((int32_t) round(q3)) ||
        !run())
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ARM_JOINT_SETTLE_DELAY_MS));

    if (!CANFD0_Operation_5((int32_t) round(q5)) || !run())
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ARM_JOINT_SETTLE_DELAY_MS));

    g_arm_solved_angles_deg[0] = q1;
    g_arm_solved_angles_deg[1] = solved_q2;
    g_arm_solved_angles_deg[2] = q3;
    g_arm_solved_angles_deg[3] = 0.0;
    g_arm_solved_angles_deg[4] = q5;
    g_arm_solved_angle_valid_mask = 0x1FU;
    arm_publish_telemetry();
    return true;
}

static bool arm_move_to_drop_pose(void)
{
    /* This is a calibrated joint pose, not an XYZ target.  Axis 4 is
     * mechanically locked at zero and must not receive a motion command. */
    if (!CANFD0_Operation_1(ARM_DROP_AXIS_1_DEG) ||
        !CANFD0_Operation_2(ARM_DROP_AXIS_2_DEG) ||
        !CANFD0_Operation_3(ARM_DROP_AXIS_3_DEG) ||
        !run())
    {
        return false;
    }

    if (!CANFD0_Operation_5(ARM_DROP_AXIS_5_DEG) || !run())
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ARM_JOINT_SETTLE_DELAY_MS));

    g_arm_solved_angles_deg[0] = ARM_DROP_AXIS_1_DEG;
    g_arm_solved_angles_deg[1] = ARM_DROP_AXIS_2_DEG;
    g_arm_solved_angles_deg[2] = ARM_DROP_AXIS_3_DEG;
    g_arm_solved_angles_deg[3] = ARM_DROP_AXIS_4_DEG;
    g_arm_solved_angles_deg[4] = ARM_DROP_AXIS_5_DEG;
    g_arm_solved_angle_valid_mask = 0x1FU;
    arm_publish_telemetry();
    return true;
}

static bool arm_pick_and_place(handeye_arm_point_t const * p_pick_point)
{
    uint32_t claw_completion_snapshot;

    if (!arm_move_to_point(p_pick_point))
    {
        (void) arm_move_to_zero();
        return false;
    }

    /* Debug mode deliberately stops at the solved detection point.  The
     * normal gripper, transfer, drop, and return-to-zero stages are skipped. */
    if (g_arm_debug_stop_at_target)
    {
        return true;
    }

    /* Claw commands use sync flag 0 and execute immediately; do not call
     * run(), which is only needed to trigger synchronized joint commands. */
    claw_completion_snapshot = CANFD0_Claw_Completion_Snapshot();
    if (!Claw_Control() ||
        !CANFD0_Wait_Claw_Complete(claw_completion_snapshot,
                                   ARM_CLAW_TIMEOUT_MS))
    {
        claw_completion_snapshot = CANFD0_Claw_Completion_Snapshot();
        if (Claw_Open())
        {
            (void) CANFD0_Wait_Claw_Complete(claw_completion_snapshot,
                                             ARM_CLAW_TIMEOUT_MS);
        }
        (void) arm_move_to_zero();
        return false;
    }

    /* Hold the fruit briefly after the gripper reports completion before
     * starting the transfer motion. */
    vTaskDelay(pdMS_TO_TICKS(ARM_CLAW_SETTLE_DELAY_MS));

    if (!arm_move_to_drop_pose())
    {
        claw_completion_snapshot = CANFD0_Claw_Completion_Snapshot();
        if (Claw_Open())
        {
            (void) CANFD0_Wait_Claw_Complete(claw_completion_snapshot,
                                             ARM_CLAW_TIMEOUT_MS);
        }
        (void) arm_move_to_zero();
        return false;
    }

    bool const claw_opened = Claw_Open();

    /* Keep the arm at the drop pose for a fixed release interval. Do not wait
     * for the optional completion reply here, otherwise a driver configured
     * without Reached responses can delay the return-to-zero by 8 seconds. */
    if (claw_opened)
    {
        vTaskDelay(pdMS_TO_TICKS(ARM_CLAW_OPEN_DELAY_MS));
    }

    /* Return the arm links to their standby positions, but restore axis 5 to
     * the angle selected when the current Task was opened (-30 or 80 deg). */
    bool const arm_ready = arm_return_to_task_pose();
    return claw_opened && arm_ready;
}

static bool arm_return_to_task_pose(void)
{
    int32_t const joint5_angle_deg = g_task_joint5_angle_deg;

    /* Complete a real all-axis ZERO first. Only after that movement settles
     * may axis 5 rotate to the standby angle selected by the current Task. */
    if (!arm_move_to_zero())
    {
        return false;
    }

    return arm_prepare_task(joint5_angle_deg);
}

static bool arm_move_to_zero(void)
{
    if (!CANFD0_Operation_1(0) ||
        !CANFD0_Operation_2(0) ||
        !CANFD0_Operation_3(0) ||
        !CANFD0_Operation_5(0) ||
        !run())
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ARM_JOINT_SETTLE_DELAY_MS));

    for (uint32_t i = 0U; i < IPC_ARM_TELEMETRY_AXIS_COUNT; i++)
    {
        g_arm_solved_angles_deg[i] = 0.0;
    }
    g_arm_solved_angle_valid_mask = 0x1FU;
    arm_publish_telemetry();
    return true;
}

static bool arm_prepare_task(int32_t joint5_angle_deg)
{
    if (!CANFD0_Operation_5(joint5_angle_deg) || !run())
    {
        return false;
    }
    g_arm_solved_angles_deg[4] = (double) joint5_angle_deg;
    g_arm_solved_angle_valid_mask |= 1UL << 4U;
    arm_publish_telemetry();
    return true;
}

static bool arm_set_axis_angle(uint8_t axis, int32_t angle_deg)
{
    bool queued;

    switch (axis)
    {
        case 1U: queued = CANFD0_Operation_1(angle_deg); break;
        case 2U: queued = CANFD0_Operation_2(angle_deg); break;
        case 3U: queued = CANFD0_Operation_3(angle_deg); break;
        case 4U: queued = CANFD0_Operation_4(angle_deg); break;
        case 5U: queued = CANFD0_Operation_5(angle_deg); break;
        default: return false;
    }

    if (!queued || !run())
    {
        return false;
    }

    g_arm_solved_angles_deg[axis - 1U] = (double) angle_deg;
    g_arm_solved_angle_valid_mask |= 1UL << (axis - 1U);
    arm_publish_telemetry();
    return true;
}

static bool arm_telemetry_send_word(uint32_t word)
{
    TickType_t const start_tick = xTaskGetTickCount();
    fsp_err_t err;

    do
    {
        err = g_ipc0.p_api->messageSend(g_ipc0.p_ctrl, word);
        if (FSP_ERR_OVERFLOW == err)
        {
            if ((xTaskGetTickCount() - start_tick) >=
                pdMS_TO_TICKS(ARM_TELEMETRY_IPC_TIMEOUT_MS))
            {
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    } while (FSP_ERR_OVERFLOW == err);

    if (FSP_SUCCESS != err)
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ARM_TELEMETRY_IPC_GAP_MS));
    return true;
}

static void arm_publish_telemetry(void)
{
    int32_t angles_0p1deg[IPC_ARM_TELEMETRY_AXIS_COUNT];
    uint32_t const valid_mask = g_arm_solved_angle_valid_mask;
    int32_t x_0p1mm = 0;
    int32_t y_0p1mm = 0;
    int32_t z_0p1mm = 0;

    for (uint32_t i = 0U; i < IPC_ARM_TELEMETRY_AXIS_COUNT; i++)
    {
        angles_0p1deg[i] = (int32_t) round(g_arm_solved_angles_deg[i] * 10.0);
    }

    if ((valid_mask & 0x07U) == 0x07U)
    {
        Pose pose;
        double const q1 = g_arm_solved_angles_deg[0] * ARM_DEG_TO_RAD;
        double const q2 = g_arm_solved_angles_deg[1] * ARM_DEG_TO_RAD;
        double const q3 = g_arm_solved_angles_deg[2] * ARM_DEG_TO_RAD;

        forward_kinematics_5dof(q1, q2, q3, &pose);
        x_0p1mm = (int32_t) round(pose.x * 10.0);
        y_0p1mm = (int32_t) round(pose.y * 10.0);
        z_0p1mm = (int32_t) round(pose.z * 10.0);
    }

    if (!arm_telemetry_send_word(IPC_ARM_TELEMETRY_BEGIN) ||
        !arm_telemetry_send_word(valid_mask))
    {
        return;
    }

    for (uint32_t i = 0U; i < IPC_ARM_TELEMETRY_AXIS_COUNT; i++)
    {
        if (!arm_telemetry_send_word((uint32_t) angles_0p1deg[i]))
        {
            return;
        }
    }

    (void) (arm_telemetry_send_word((uint32_t) x_0p1mm) &&
            arm_telemetry_send_word((uint32_t) y_0p1mm) &&
            arm_telemetry_send_word((uint32_t) z_0p1mm) &&
            arm_telemetry_send_word(IPC_ARM_TELEMETRY_END));
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
    static ipc_manual_coordinate_t manual_coordinate;

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

    if (IPC_MANUAL_COORDINATE_BEGIN == p_args->message)
    {
        receive_state = IPC_RX_READ_MANUAL_X;
        return;
    }

    if (IPC_ARM_ZERO_COMMAND == p_args->message)
    {
        g_axis_angle_queue_read = g_axis_angle_queue_write;
        g_claw_request_pending = false;
        g_arm_zero_request_pending = true;
        ipc_coordinate_rx_reset(&receive_state);
        return;
    }

    if ((IPC_CLAW_CLOSE_COMMAND == p_args->message) ||
        (IPC_CLAW_OPEN_COMMAND == p_args->message))
    {
        g_claw_open_requested = (IPC_CLAW_OPEN_COMMAND == p_args->message);
        g_claw_request_pending = true;
        ipc_coordinate_rx_reset(&receive_state);
        return;
    }

    if (IPC_ARM_TELEMETRY_REQUEST == p_args->message)
    {
        g_arm_telemetry_request_pending = true;
        ipc_coordinate_rx_reset(&receive_state);
        return;
    }

    if ((IPC_ARM_DEBUG_STOP_DISABLE == p_args->message) ||
        (IPC_ARM_DEBUG_STOP_ENABLE == p_args->message))
    {
        g_arm_debug_stop_at_target =
            (IPC_ARM_DEBUG_STOP_ENABLE == p_args->message);
        ipc_coordinate_rx_reset(&receive_state);
        return;
    }

    if (IPC_CLAW_CURRENT_COMMAND_PREFIX ==
        (p_args->message & IPC_CLAW_CURRENT_COMMAND_MASK))
    {
        uint32_t const current_ma =
            p_args->message & IPC_CLAW_CURRENT_VALUE_MASK;

        if (current_ma <= IPC_CLAW_CURRENT_MAX_MA)
        {
            Claw_SetCloseCurrent((uint16_t) current_ma);
        }
        ipc_coordinate_rx_reset(&receive_state);
        return;
    }

    if (IPC_TASK1_JOINT5_COMMAND == p_args->message)
    {
        g_task_joint5_angle_deg = ARM_TASK1_JOINT_5_DEG;
        g_task_joint5_request_pending = true;
        ipc_coordinate_rx_reset(&receive_state);
        return;
    }

    if (IPC_TASK2_JOINT5_COMMAND == p_args->message)
    {
        g_task_joint5_angle_deg = ARM_TASK2_JOINT_5_DEG;
        g_task_joint5_request_pending = true;
        ipc_coordinate_rx_reset(&receive_state);
        return;
    }

    if (IPC_AXIS_ANGLE_COMMAND_PREFIX ==
        (p_args->message & IPC_AXIS_ANGLE_COMMAND_MASK))
    {
        uint32_t const axis = p_args->message & IPC_AXIS_ANGLE_AXIS_MASK;
        uint32_t const encoded_angle =
            (p_args->message & IPC_AXIS_ANGLE_VALUE_MASK) >>
            IPC_AXIS_ANGLE_VALUE_SHIFT;
        int32_t const angle_deg = (int32_t) encoded_angle +
                                  IPC_AXIS_ANGLE_MIN_DEG;
        uint32_t const next = (g_axis_angle_queue_write + 1U) %
                              ARM_AXIS_ANGLE_QUEUE_LENGTH;

        if ((axis >= 1U) && (axis <= ARM_AXIS_COUNT) &&
            (angle_deg >= IPC_AXIS_ANGLE_MIN_DEG) &&
            (angle_deg <= IPC_AXIS_ANGLE_MAX_DEG) &&
            (next != g_axis_angle_queue_read))
        {
            g_axis_angle_queue[g_axis_angle_queue_write] = p_args->message;
            g_axis_angle_queue_write = next;
        }
        ipc_coordinate_rx_reset(&receive_state);
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

        case IPC_RX_READ_MANUAL_X:
        {
            manual_coordinate.x_0p1mm = (int32_t) p_args->message;
            receive_state = IPC_RX_READ_MANUAL_Y;
            break;
        }

        case IPC_RX_READ_MANUAL_Y:
        {
            manual_coordinate.y_0p1mm = (int32_t) p_args->message;
            receive_state = IPC_RX_READ_MANUAL_Z;
            break;
        }

        case IPC_RX_READ_MANUAL_Z:
        {
            manual_coordinate.z_0p1mm = (int32_t) p_args->message;
            receive_state = IPC_RX_WAIT_MANUAL_END;
            break;
        }

        case IPC_RX_WAIT_MANUAL_END:
        {
            if ((IPC_MANUAL_COORDINATE_END == p_args->message) &&
                (manual_coordinate.x_0p1mm >=
                     IPC_MANUAL_COORDINATE_MIN_0P1MM) &&
                (manual_coordinate.x_0p1mm <=
                     IPC_MANUAL_COORDINATE_MAX_0P1MM) &&
                (manual_coordinate.y_0p1mm >=
                     IPC_MANUAL_COORDINATE_MIN_0P1MM) &&
                (manual_coordinate.y_0p1mm <=
                     IPC_MANUAL_COORDINATE_MAX_0P1MM) &&
                (manual_coordinate.z_0p1mm >=
                     IPC_MANUAL_COORDINATE_MIN_0P1MM) &&
                (manual_coordinate.z_0p1mm <=
                     IPC_MANUAL_COORDINATE_MAX_0P1MM))
            {
                g_manual_coordinate_request.x_0p1mm =
                    manual_coordinate.x_0p1mm;
                g_manual_coordinate_request.y_0p1mm =
                    manual_coordinate.y_0p1mm;
                g_manual_coordinate_request.z_0p1mm =
                    manual_coordinate.z_0p1mm;
                g_manual_coordinate_request_pending = true;
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
	/* A physical board reset restarts CPU1 while the motor drivers may still
	 * be powered. Give them time to become ready, then perform the same ZERO
	 * action as the screen button. */
	vTaskDelay(pdMS_TO_TICKS(ARM_RESET_ZERO_DELAY_MS));
	(void) arm_move_to_zero();
#endif
	while (1)
	{
#if !DM_COORDINATE_UART_ENABLE
        bool arm_zero_requested;
        bool claw_requested;
        bool claw_open_requested = false;
        bool task_joint5_requested;
        int32_t task_joint5_angle_deg = 0;
        bool axis_angle_requested;
        uint32_t axis_angle_command = 0U;
        uint32_t axis_angle_queue_index = 0U;
        bool arm_telemetry_requested;

        taskENTER_CRITICAL();
        arm_zero_requested = g_arm_zero_request_pending;
        g_arm_zero_request_pending = false;
        if (arm_zero_requested)
        {
            g_claw_request_pending = false;
        }
        claw_requested = !arm_zero_requested && g_claw_request_pending;
        if (claw_requested)
        {
            claw_open_requested = g_claw_open_requested;
            g_claw_request_pending = false;
        }
        task_joint5_requested = !arm_zero_requested && !claw_requested &&
                                g_task_joint5_request_pending;
        if (task_joint5_requested)
        {
            task_joint5_angle_deg = g_task_joint5_angle_deg;
            g_task_joint5_request_pending = false;
        }
        axis_angle_requested = !arm_zero_requested && !claw_requested &&
                               !task_joint5_requested &&
                               (g_axis_angle_queue_read != g_axis_angle_queue_write);
        if (axis_angle_requested)
        {
            /* Keep the entry queued until both CAN packets and the sync
             * trigger have actually been transmitted. */
            axis_angle_queue_index = g_axis_angle_queue_read;
            axis_angle_command = g_axis_angle_queue[axis_angle_queue_index];
        }
        arm_telemetry_requested = !arm_zero_requested &&
                                  !claw_requested &&
                                  !task_joint5_requested &&
                                  !axis_angle_requested &&
                                  g_arm_telemetry_request_pending;
        if (arm_telemetry_requested)
        {
            g_arm_telemetry_request_pending = false;
        }
        taskEXIT_CRITICAL();

        if (arm_zero_requested)
        {
            (void) arm_move_to_zero();
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        if (claw_requested)
        {
            if (claw_open_requested)
            {
                (void) Claw_Open();
            }
            else
            {
                (void) Claw_Control();
            }
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        if (task_joint5_requested)
        {
            (void) arm_prepare_task(task_joint5_angle_deg);
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        if (axis_angle_requested)
        {
            uint8_t const axis = (uint8_t) (axis_angle_command &
                                            IPC_AXIS_ANGLE_AXIS_MASK);
            uint32_t const encoded_angle =
                (axis_angle_command & IPC_AXIS_ANGLE_VALUE_MASK) >>
                IPC_AXIS_ANGLE_VALUE_SHIFT;
            int32_t const angle_deg = (int32_t) encoded_angle +
                                      IPC_AXIS_ANGLE_MIN_DEG;

            if (arm_set_axis_angle(axis, angle_deg))
            {
                taskENTER_CRITICAL();
                if ((g_axis_angle_queue_read == axis_angle_queue_index) &&
                    (g_axis_angle_queue[axis_angle_queue_index] == axis_angle_command))
                {
                    g_axis_angle_queue_read = (g_axis_angle_queue_read + 1U) %
                                              ARM_AXIS_ANGLE_QUEUE_LENGTH;
                }
                taskEXIT_CRITICAL();
            }
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        if (arm_telemetry_requested)
        {
            arm_publish_telemetry();
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }
#endif

        bool manual_coordinate_requested;
        ipc_manual_coordinate_t manual_coordinate;

        taskENTER_CRITICAL();
        manual_coordinate_requested = g_manual_coordinate_request_pending;
        if (manual_coordinate_requested)
        {
            manual_coordinate.x_0p1mm =
                g_manual_coordinate_request.x_0p1mm;
            manual_coordinate.y_0p1mm =
                g_manual_coordinate_request.y_0p1mm;
            manual_coordinate.z_0p1mm =
                g_manual_coordinate_request.z_0p1mm;
            g_manual_coordinate_request_pending = false;
        }
        taskEXIT_CRITICAL();

        if (manual_coordinate_requested)
        {
            handeye_arm_point_t arm_point;

            arm_point.x_mm = (double) manual_coordinate.x_0p1mm / 10.0;
            arm_point.y_mm = (double) manual_coordinate.y_0p1mm / 10.0;
            arm_point.z_mm = (double) manual_coordinate.z_0p1mm / 10.0;
#if DM_COORDINATE_UART_ENABLE
            if (ipc_arm_point_publish(&arm_point, 0U, 0U))
            {
                xTaskNotifyGive(Uart_dm_thread);
            }
#else
            (void) arm_move_to_point(&arm_point);
#endif
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

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
                (void) arm_pick_and_place(&arm_point);
#endif
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
	}
}

