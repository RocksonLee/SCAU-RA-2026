#ifndef UART_COORDINATE_PROTOCOL_H
#define UART_COORDINATE_PROTOCOL_H

#include <stdint.h>

/*
 * Coordinate output selection:
 *   0: keep the original inverse-kinematics + CAN motor control path.
 *   1: send calibrated arm coordinates to the other board over SCI0 UART.
 */
#define DM_COORDINATE_UART_ENABLE            (1) //0就是坐标逆解后，继续通过 CAN 控制电机。 1就是不执行这组 CAN 动作，改由 SCI0/UART0 将坐标发给 DM_MC02。

#if ((DM_COORDINATE_UART_ENABLE != 0) && (DM_COORDINATE_UART_ENABLE != 1))
 #error "DM_COORDINATE_UART_ENABLE must be 0 or 1."
#endif

/* Fixed-length DM coordinate frame, version 1. */
#define DM_UART_FRAME_SOF0                   (0xAAU)
#define DM_UART_FRAME_SOF1                   (0x55U)
#define DM_UART_PROTOCOL_VERSION             (0x01U)
#define DM_UART_MESSAGE_COORDINATE           (0x01U)
#define DM_UART_COORDINATE_SCALE_PER_MM      (100.0)
#define DM_UART_COORDINATE_FRAME_SIZE        (24U)
#define DM_UART_COORDINATE_CRC_OFFSET        (22U)

/* Byte offsets in a coordinate frame. Multi-byte fields are little-endian. */
#define DM_UART_OFFSET_SOF0                  (0U)
#define DM_UART_OFFSET_SOF1                  (1U)
#define DM_UART_OFFSET_VERSION               (2U)
#define DM_UART_OFFSET_MESSAGE_TYPE          (3U)
#define DM_UART_OFFSET_PAIR_ID               (4U)
#define DM_UART_OFFSET_CLASS_ID              (8U)
#define DM_UART_OFFSET_FLAGS                 (9U)
#define DM_UART_OFFSET_X                     (10U)
#define DM_UART_OFFSET_Y                     (14U)
#define DM_UART_OFFSET_Z                     (18U)

#endif /* UART_COORDINATE_PROTOCOL_H */
