#ifndef IPC_DETECTION_PROTOCOL_H
#define IPC_DETECTION_PROTOCOL_H

#include <stdint.h>

#define IPC_COORDINATE_BATCH_BEGIN      (0x33444242U) /* "3DBB" */
#define IPC_COORDINATE_BATCH_END        (0x33444245U) /* "3DBE" */
#define IPC_ARM_ZERO_COMMAND            (0x5A45524FU) /* "ZERO" */
#define IPC_COORDINATE_BATCH_MAX_ITEMS  (3U)

typedef struct st_ipc_camera_coordinate_item
{
    uint32_t class_id;
    int32_t  top_x;
    int32_t  top_y;
    int32_t  side_x;
    int32_t  side_y;
} ipc_camera_coordinate_item_t;

typedef struct st_ipc_camera_coordinate_batch
{
    uint32_t batch_id;
    uint32_t count;
    ipc_camera_coordinate_item_t items[IPC_COORDINATE_BATCH_MAX_ITEMS];
} ipc_camera_coordinate_batch_t;

typedef struct st_ipc_camera_coordinate_pair
{
    uint32_t pair_id;
    uint32_t class_id;
    int32_t  top_x;
    int32_t  top_y;
    int32_t  side_x;
    int32_t  side_y;
} ipc_camera_coordinate_pair_t;

#endif /* IPC_DETECTION_PROTOCOL_H */
