#include "ipc_detection_tx.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "hal_data.h"
#include "ipc_detection_protocol.h"

#define IPC_DETECTION_SEND_TIMEOUT_MS (100U)

static bool ipc_detection_send_word(uint32_t word, TickType_t packet_start)
{
    fsp_err_t err;

    do
    {
        err = g_ipc0.p_api->messageSend(g_ipc0.p_ctrl, word);

        if (FSP_ERR_OVERFLOW == err)
        {
            if ((xTaskGetTickCount() - packet_start) >=
                pdMS_TO_TICKS(IPC_DETECTION_SEND_TIMEOUT_MS))
            {
                return false;
            }

            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    } while (FSP_ERR_OVERFLOW == err);

    return FSP_SUCCESS == err;
}

bool ipc_detection_send_coordinate_batch(ipc_camera_coordinate_batch_t const * p_batch)
{
    if ((NULL == p_batch) ||
        (0U == p_batch->count) ||
        (p_batch->count > IPC_COORDINATE_BATCH_MAX_ITEMS))
    {
        return false;
    }

    TickType_t const packet_start = xTaskGetTickCount();

    if (!ipc_detection_send_word(IPC_COORDINATE_BATCH_BEGIN, packet_start) ||
        !ipc_detection_send_word(p_batch->batch_id, packet_start) ||
        !ipc_detection_send_word(p_batch->count, packet_start))
    {
        return false;
    }

    for (uint32_t i = 0U; i < p_batch->count; i++)
    {
        ipc_camera_coordinate_item_t const * p_item = &p_batch->items[i];

        if (!ipc_detection_send_word(p_item->class_id, packet_start) ||
            !ipc_detection_send_word((uint32_t) p_item->top_x, packet_start) ||
            !ipc_detection_send_word((uint32_t) p_item->top_y, packet_start) ||
            !ipc_detection_send_word((uint32_t) p_item->side_x, packet_start) ||
            !ipc_detection_send_word((uint32_t) p_item->side_y, packet_start))
        {
            return false;
        }
    }

    return ipc_detection_send_word(IPC_COORDINATE_BATCH_END, packet_start);
}
