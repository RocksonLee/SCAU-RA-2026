#include "ipc_detection_tx.h"

#include <stddef.h>

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
