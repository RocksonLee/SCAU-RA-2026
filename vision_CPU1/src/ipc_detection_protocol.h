#ifndef IPC_DETECTION_PROTOCOL_H
#define IPC_DETECTION_PROTOCOL_H

#include <stdint.h>

#define IPC_DETECTION_BEGIN             (0x44455442U) /* "DETB" */
#define IPC_DETECTION_END               (0x44455445U) /* "DETE" */
#define IPC_DETECTION_MAX_RESULTS       (3U)
#define IPC_DETECTION_WORDS_PER_RESULT  (3U)

#endif /* IPC_DETECTION_PROTOCOL_H */
