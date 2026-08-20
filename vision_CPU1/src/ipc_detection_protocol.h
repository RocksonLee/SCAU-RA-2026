#ifndef IPC_DETECTION_PROTOCOL_H
#define IPC_DETECTION_PROTOCOL_H

#include <stdint.h>

#define IPC_COORDINATE_BATCH_BEGIN      (0x33444242U) /* "3DBB" */
#define IPC_COORDINATE_BATCH_END        (0x33444245U) /* "3DBE" */
#define IPC_ARM_ZERO_COMMAND            (0x5A45524FU) /* "ZERO" */
#define IPC_CLAW_CLOSE_COMMAND          (0x434C4F53U) /* "CLOS" */
#define IPC_CLAW_OPEN_COMMAND           (0x4F50454EU) /* "OPEN" */
#define IPC_ARM_TELEMETRY_REQUEST       (0x41524D3FU) /* "ARM?" */
#define IPC_ARM_TELEMETRY_BEGIN         (0x41524D42U) /* "ARMB" */
#define IPC_ARM_TELEMETRY_END           (0x41524D45U) /* "ARME" */
#define IPC_ARM_TELEMETRY_AXIS_COUNT    (5U)
#define IPC_TASK1_JOINT5_COMMAND        (0x54314A35U) /* "T1J5" */
#define IPC_TASK2_JOINT5_COMMAND        (0x54324A35U) /* "T2J5" */
#define IPC_ARM_DEBUG_STOP_DISABLE      (0x44535430U) /* "DST0" */
#define IPC_ARM_DEBUG_STOP_ENABLE       (0x44535431U) /* "DST1" */
#define IPC_CALIBRATION_FIRST_COMMAND   (0x434D5631U) /* "CMV1" */
#define IPC_CALIBRATION_MOVE_BEGIN      (0x434D5642U) /* "CMVB" */
#define IPC_CALIBRATION_MOVE_END        (0x434D5645U) /* "CMVE" */
#define IPC_CALIBRATION_RESULT_BEGIN    (0x43525342U) /* "CRSB" */
#define IPC_CALIBRATION_RESULT_END      (0x43525345U) /* "CRSE" */
#define IPC_CALIBRATION_CONFIG_BEGIN    (0x43434642U) /* "CCFB" */
#define IPC_CALIBRATION_CONFIG_END      (0x43434645U) /* "CCFE" */
#define IPC_CLAW_CURRENT_COMMAND_PREFIX (0x434D0000U) /* "CM" + current in mA */
#define IPC_CLAW_CURRENT_COMMAND_MASK   (0xFFFF0000U)
#define IPC_CLAW_CURRENT_VALUE_MASK     (0x0000FFFFU)
#define IPC_CLAW_CURRENT_MAX_MA         (5000U)
#define IPC_AXIS_ANGLE_COMMAND_PREFIX   (0x414E0000U) /* "AN" + packed payload */
#define IPC_AXIS_ANGLE_COMMAND_MASK     (0xFFFF0000U)
#define IPC_AXIS_ANGLE_AXIS_MASK        (0x00000007U)
#define IPC_AXIS_ANGLE_VALUE_SHIFT      (3U)
#define IPC_AXIS_ANGLE_VALUE_MASK       (0x00000FF8U)
#define IPC_AXIS_ANGLE_MIN_DEG          (-180)
#define IPC_AXIS_ANGLE_MAX_DEG          (180)
#define IPC_COORDINATE_BATCH_MAX_ITEMS  (3U)
#define IPC_CALIBRATION_FIRST_SEQUENCE  (1U)
#define IPC_CALIBRATION_FIRST_X_0P1MM   (1350)
#define IPC_CALIBRATION_FIRST_Y_0P1MM   (2550)
#define IPC_CALIBRATION_FIRST_Z_0P1MM   (3250)

typedef struct st_ipc_handeye_calibration
{
    float top_z_low_mm;
    float top_z_high_mm;
    float top_h_low[3][3];
    float top_h_high[3][3];
    float side_z_slope_mm_px;
    float side_z_offset_mm;
} ipc_handeye_calibration_t;

#define IPC_CALIBRATION_CONFIG_WORDS \
    ((uint32_t) (sizeof(ipc_handeye_calibration_t) / sizeof(uint32_t)))

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
