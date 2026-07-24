#ifndef IPC_DETECTION_TX_H
#define IPC_DETECTION_TX_H

#include <stdbool.h>
#include <stdint.h>

#include "app_detection.h"

bool ipc_detection_send_results(app_detection_result_t const * p_results,
                                uint32_t                         result_count);

#endif /* IPC_DETECTION_TX_H */
