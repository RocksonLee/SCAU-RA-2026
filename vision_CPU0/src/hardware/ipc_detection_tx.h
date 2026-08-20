#ifndef IPC_DETECTION_TX_H
#define IPC_DETECTION_TX_H

#include <stdbool.h>

#include "ipc_detection_protocol.h"

bool ipc_detection_send_coordinate_batch(ipc_camera_coordinate_batch_t const * p_batch);
bool ipc_detection_send_arm_zero(void);
bool ipc_detection_send_claw(bool open);
bool ipc_detection_send_task_joint5(int32_t angle_deg);
bool ipc_detection_send_arm_debug_stop(bool enabled);
bool ipc_detection_send_claw_current(uint16_t current_ma);
bool ipc_detection_send_axis_angle(uint8_t axis, int32_t angle_deg);
bool ipc_detection_send_manual_coordinate(int32_t x_0p1mm,
                                          int32_t y_0p1mm,
                                          int32_t z_0p1mm);
bool ipc_detection_send_arm_telemetry_request(void);

#endif /* IPC_DETECTION_TX_H */
