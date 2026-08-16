#ifndef IPC_DETECTION_TX_H
#define IPC_DETECTION_TX_H

#include <stdbool.h>

#include "ipc_detection_protocol.h"

bool ipc_detection_send_coordinate_batch(ipc_camera_coordinate_batch_t const * p_batch);
bool ipc_detection_send_arm_zero(void);
bool ipc_detection_send_task_joint5(int32_t angle_deg);

#endif /* IPC_DETECTION_TX_H */
