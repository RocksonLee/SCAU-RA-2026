#ifndef CAMERA_STREAM_H
#define CAMERA_STREAM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void camera_stream_task(void);
bool camera_debug_uart_init(void);
bool camera_debug_send_text(char const * p_text);
void camera_calibration_notify_arm_result(uint32_t sequence,
                                          bool success,
                                          int32_t x_0p1mm,
                                          int32_t y_0p1mm,
                                          int32_t z_0p1mm);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_STREAM_H */
