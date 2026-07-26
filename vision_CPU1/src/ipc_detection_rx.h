#ifndef IPC_DETECTION_RX_H
#define IPC_DETECTION_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "ipc_detection_protocol.h"

typedef struct st_ipc_detection_result
{
    uint32_t class_id;
    int32_t  x;
    int32_t  y;
} ipc_detection_result_t;

/*
 * Copies the latest complete detection packet into p_results. The packet is
 * retained when result_capacity is too small; in that case p_result_count is
 * set to the required capacity and the function returns false.
 *
 * This function is intended to be called from a FreeRTOS task, not from an
 * interrupt. It returns false when no new complete packet is available.
 */
bool ipc_detection_take_results(ipc_detection_result_t * p_results,
                                uint32_t                   result_capacity,
                                uint32_t                 * p_result_count);

#endif /* IPC_DETECTION_RX_H */
