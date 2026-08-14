#ifndef IPC_DETECTION_TX_H
#define IPC_DETECTION_TX_H

#include <stdbool.h>

#include "ipc_detection_protocol.h"

bool ipc_detection_send_coordinate_batch(ipc_camera_coordinate_batch_t const * p_batch);

#endif /* IPC_DETECTION_TX_H */
