#ifndef CAMERA_STREAM_H
#define CAMERA_STREAM_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void camera_stream_task(void);
bool camera_debug_uart_init(void);
bool camera_debug_send_text(char const * p_text);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_STREAM_H */
