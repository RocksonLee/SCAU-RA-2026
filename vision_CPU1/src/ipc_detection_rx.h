#ifndef IPC_DETECTION_RX_H
#define IPC_DETECTION_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/handeye_transform.h"
#include "ipc_detection_protocol.h"

bool ipc_coordinate_pair_take(ipc_camera_coordinate_pair_t * p_pair);

bool ipc_arm_point_take(handeye_arm_point_t * p_arm_point,
                        uint32_t            * p_pair_id,
                        uint32_t            * p_class_id);

#endif /* IPC_DETECTION_RX_H */
