#include "ipc_detection_tx.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "hal_data.h"
#include "ipc_detection_protocol.h"

#if APP_DETECTION_MAX_RESULTS != IPC_DETECTION_MAX_RESULTS
 #error "Detection result capacity does not match the IPC protocol."
#endif

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

bool ipc_detection_send_results(app_detection_result_t const * p_results,
                                uint32_t                         result_count)
{
    if ((NULL == p_results) || (result_count > IPC_DETECTION_MAX_RESULTS))
    {
        return false;
    }

    TickType_t const packet_start = xTaskGetTickCount();

    if (!ipc_detection_send_word(IPC_DETECTION_BEGIN, packet_start))
    {
        return false;
    }

    if (!ipc_detection_send_word(result_count, packet_start))
    {
        return false;
    }

    for (uint32_t i = 0U; i < result_count; i++)
    {
        if (!ipc_detection_send_word(p_results[i].class_id, packet_start) ||
            !ipc_detection_send_word((uint32_t) p_results[i].x, packet_start) ||
            !ipc_detection_send_word((uint32_t) p_results[i].y, packet_start))
        {
            return false;
        }
    }

    return ipc_detection_send_word(IPC_DETECTION_END, packet_start);
}
