#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool uart_debug_send_text(char const * p_text);
void uart_debug_set_logs_enabled(bool enabled);
bool uart_debug_logs_enabled(void);
bool uart_debug_request_frame_dump(uint8_t const * p_rgb565_frame,
                                   uint32_t        frame_bytes);

#ifdef __cplusplus
}
#endif

#endif /* UART_DEBUG_H */
