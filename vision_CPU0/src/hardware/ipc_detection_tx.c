#include "ipc_detection_tx.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "hal_data.h"
#include "ipc_detection_protocol.h"

#define IPC_DETECTION_WORD_TIMEOUT_MS (500U)
#define IPC_DETECTION_WORD_GAP_MS     (2U)

static bool ipc_detection_send_word(uint32_t word)
{
    fsp_err_t err;
    TickType_t const word_start = xTaskGetTickCount();

    do
    {
        err = g_ipc0.p_api->messageSend(g_ipc0.p_ctrl, word);

        if (FSP_ERR_OVERFLOW == err)
        {
            if ((xTaskGetTickCount() - word_start) >=
                pdMS_TO_TICKS(IPC_DETECTION_WORD_TIMEOUT_MS))
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

    /* Give CPU1 time to pop this word before the next FIFO write. */
    vTaskDelay(pdMS_TO_TICKS(IPC_DETECTION_WORD_GAP_MS));
    return true;
}

bool ipc_detection_send_coordinate_batch(ipc_camera_coordinate_batch_t const * p_batch)
{
    if ((NULL == p_batch) ||
        (0U == p_batch->count) ||
        (p_batch->count > IPC_COORDINATE_BATCH_MAX_ITEMS))
    {
        return false;
    }

    if (!ipc_detection_send_word(IPC_COORDINATE_BATCH_BEGIN) ||
        !ipc_detection_send_word(p_batch->batch_id) ||
        !ipc_detection_send_word(p_batch->count))
    {
        return false;
    }

    for (uint32_t i = 0U; i < p_batch->count; i++)
    {
        ipc_camera_coordinate_item_t const * p_item = &p_batch->items[i];

        if (!ipc_detection_send_word(p_item->class_id) ||
            !ipc_detection_send_word((uint32_t) p_item->top_x) ||
            !ipc_detection_send_word((uint32_t) p_item->top_y) ||
            !ipc_detection_send_word((uint32_t) p_item->side_x) ||
            !ipc_detection_send_word((uint32_t) p_item->side_y))
        {
            return false;
        }
    }

    return ipc_detection_send_word(IPC_COORDINATE_BATCH_END);
}

bool ipc_detection_send_arm_zero(void)
{
    return ipc_detection_send_word(IPC_ARM_ZERO_COMMAND);
}

bool ipc_detection_send_claw(bool open)
{
    return ipc_detection_send_word(open ? IPC_CLAW_OPEN_COMMAND :
                                          IPC_CLAW_CLOSE_COMMAND);
}

bool ipc_detection_send_task_joint5(int32_t angle_deg)
{
    if (-30 == angle_deg)
    {
        return ipc_detection_send_word(IPC_TASK1_JOINT5_COMMAND);
    }

    if (80 == angle_deg)
    {
        return ipc_detection_send_word(IPC_TASK2_JOINT5_COMMAND);
    }

    return false;
}

bool ipc_detection_send_arm_debug_stop(bool enabled)
{
    return ipc_detection_send_word(enabled ? IPC_ARM_DEBUG_STOP_ENABLE :
                                              IPC_ARM_DEBUG_STOP_DISABLE);
}

bool ipc_detection_send_claw_current(uint16_t current_ma)
{
    if (current_ma > IPC_CLAW_CURRENT_MAX_MA)
    {
        return false;
    }

    return ipc_detection_send_word(IPC_CLAW_CURRENT_COMMAND_PREFIX |
                                   (uint32_t) current_ma);
}

bool ipc_detection_send_axis_angle(uint8_t axis, int32_t angle_deg)
{
    uint32_t command;
    uint32_t encoded_angle;

    if ((axis < 1U) || (axis > 5U) ||
        (angle_deg < IPC_AXIS_ANGLE_MIN_DEG) ||
        (angle_deg > IPC_AXIS_ANGLE_MAX_DEG))
    {
        return false;
    }

    encoded_angle = (uint32_t) (angle_deg - IPC_AXIS_ANGLE_MIN_DEG);
    command = IPC_AXIS_ANGLE_COMMAND_PREFIX |
              ((encoded_angle << IPC_AXIS_ANGLE_VALUE_SHIFT) &
               IPC_AXIS_ANGLE_VALUE_MASK) |
              (uint32_t) axis;

    return ipc_detection_send_word(command);
}

bool ipc_detection_send_arm_telemetry_request(void)
{
    return ipc_detection_send_word(IPC_ARM_TELEMETRY_REQUEST);
}

bool ipc_detection_send_calibration_first(void)
{
    /* The first calibration move is intentionally one IPC word.  It must be
     * able to start with an empty camera view and cannot depend on a model
     * result or a partially received multi-word coordinate packet. */
    return ipc_detection_send_word(IPC_CALIBRATION_FIRST_COMMAND);
}

bool ipc_detection_send_calibration_move(uint32_t sequence,
                                         int32_t  x_0p1mm,
                                         int32_t  y_0p1mm,
                                         int32_t  z_0p1mm)
{
    return ipc_detection_send_word(IPC_CALIBRATION_MOVE_BEGIN) &&
           ipc_detection_send_word(sequence) &&
           ipc_detection_send_word((uint32_t) x_0p1mm) &&
           ipc_detection_send_word((uint32_t) y_0p1mm) &&
           ipc_detection_send_word((uint32_t) z_0p1mm) &&
           ipc_detection_send_word(IPC_CALIBRATION_MOVE_END);
}

bool ipc_detection_send_calibration_config(ipc_handeye_calibration_t const * p_config)
{
    if ((NULL == p_config) ||
        (0U != (sizeof(*p_config) % sizeof(uint32_t))))
    {
        return false;
    }

    if (!ipc_detection_send_word(IPC_CALIBRATION_CONFIG_BEGIN))
    {
        return false;
    }

    for (uint32_t i = 0U; i < IPC_CALIBRATION_CONFIG_WORDS; i++)
    {
        uint32_t word;
        memcpy(&word, &((uint8_t const *) p_config)[i * sizeof(word)], sizeof(word));
        if (!ipc_detection_send_word(word))
        {
            return false;
        }
    }

    return ipc_detection_send_word(IPC_CALIBRATION_CONFIG_END);
}
